# Copyright (c) 2026，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.actions import GroupAction
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    # 使能 FastDDS SHM 零拷贝环境（等价 hobot_shm.launch.py，需在容器启动前设置）
    if os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML') != '1':
        os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
        os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(
            get_package_prefix('hobot_shm'), "lib/hobot_shm/config/shm_fastdds.xml"))
        os.environ.setdefault('RMW_FASTRTPS_USE_QOS_FROM_XML', '1')
        os.environ.setdefault('ROS_DISABLE_LOANED_MESSAGES', '0')
        print("Hobot shm pkg enables zero-copy with fastrtps profiles file:",
              os.getenv('FASTRTPS_DEFAULT_PROFILES_FILE'))

    # 拷贝算法config到cwd，节点里的 config/xxx 相对路径才能解析
    mono_ocr_path = os.path.join(
        get_package_prefix('mono_ocr_detection'), "lib/mono_ocr_detection")
    print("mono_ocr_path is ", mono_ocr_path)
    os.system("cp -rn " + mono_ocr_path + "/config/. ./config/ 2>/dev/null")

    # component 形式的 4K 实时/回灌链，算法输入统一为 HbmMsg2160P nv12（/hbmem_img_2160p）：
    # usb（默认）：usb_cam(独立进程, mjpeg /image) -> 解码组件(硬解) -> OCR组件 -> web直通相机mjpeg
    # fb        ：发布组件(jpg -> nv12) -> OCR组件 -> 编码组件(jpeg /image) -> web
    # 注：publisher/codec 使用 loaned message，不能开 intra-process（humble 不支持）
    cam_type = os.getenv('CAM_TYPE', 'usb')
    print("cam_type is ", cam_type)

    image_source_launch_arg = DeclareLaunchArgument(
        'image_source',
        default_value='./config/component.jpg',
        description='fb mode: local 4k jpg image path')
    source_fps_launch_arg = DeclareLaunchArgument(
        'source_fps',
        default_value='5',
        description='fb mode: publish fps')
    camera_device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video0',
        description='usb mode: 4k camera device')
    camera_fps_launch_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='30',
        description='usb mode: camera capture framerate')
    publish_fps_launch_arg = DeclareLaunchArgument(
        'publish_fps',
        default_value='30',
        description='usb mode: camera publish rate limit (0 = publish all)')
    config_file_launch_arg = DeclareLaunchArgument(
        "mono_ocr_detection_config_file",
        default_value=TextSubstitution(text="config/mono_ocr_detection_workconfig.json"))
    dump_render_launch_arg = DeclareLaunchArgument(
        "dump_render_img",
        default_value='0',
        description='dump rendered image for debug')
    msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "mono_ocr_detection_msg_pub_topic_name",
        default_value=TextSubstitution(text="/hobot_ocr_detection"))
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='tros_container',
        description='the name of the container that launches all nodes')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='log level')

    composable_node_descriptions = []

    # 图片源节点
    if cam_type == 'fb':
        # fb：jpg -> nv12 -> HbmMsg2160P(/hbmem_img)，组件加载
        composable_node_descriptions.append(
            ComposableNode(
                package='hobot_image_publisher',
                plugin='PubNode',
                name='image_pub_node',
                parameters=[
                    {"image_source": LaunchConfiguration('image_source')},
                    {"image_format": 'jpg'},
                    {"msg_pub_topic_name": '/hbmem_img'},
                    {"out_msg_type": '2160p'},
                    {"is_shared_mem": True},
                    {"is_compressed_img_pub": False},
                    {"pub_encoding": 'nv12'},
                    {"fps": LaunchConfiguration('source_fps')},
                ],
            ),
        )
    else:
        # usb：usb_cam 组件(mjpeg /image) -> 解码组件（硬解 -> HbmMsg2160P nv12）
        composable_node_descriptions.append(
            ComposableNode(
                package='hobot_usb_cam',
                plugin='usb_cam::HobotUsbCamNode',
                name='usb_cam_node',
                parameters=[
                    {'image_width': 3840},
                    {'image_height': 2160},
                    {'framerate': LaunchConfiguration('camera_fps')},
                    {'publish_fps': LaunchConfiguration('publish_fps')},
                    {'pixel_format': 'mjpeg'},
                    {'io_method': 'mmap'},
                    {'video_device': LaunchConfiguration('device')},
                ],
            ),
        )
        composable_node_descriptions.append(
            ComposableNode(
                package='hobot_codec',
                plugin='HobotCodecNode',
                name='image_jpeg_decoder_node',
                parameters=[
                    {'sub_topic': '/image'},
                    {'in_mode': 'ros'},
                    {'in_format': 'jpeg'},
                    # ros CompressedImage 不携带分辨率，vdec 按此建输入通道
                    {'in_width': 3840},
                    {'in_height': 2160},
                    {'out_mode': 'shared_mem'},
                    {'out_msg_type': '2160p'},
                    {'out_format': 'nv12'},
                    {'pub_topic': '/hbmem_img'},
                ],
            ),
        )

    # OCR检测节点：订阅4K共享内存帧
    composable_node_descriptions.append(
        ComposableNode(
            package='mono_ocr_detection',
            plugin='MonoOcrDetectionNode',
            name='mono_ocr_detection_node',
            parameters=[
                {'feed_type': 1},
                {'is_shared_mem_sub': 1},
                {'config_file': LaunchConfiguration('mono_ocr_detection_config_file')},
                {'sharedmem_img_topic_name': '/hbmem_img'},
                {'ai_msg_pub_topic_name': LaunchConfiguration(
                    "mono_ocr_detection_msg_pub_topic_name")},
                {'dump_render_img': LaunchConfiguration('dump_render_img')},
            ],
        ),
    )

    # usb 有相机 mjpeg 直通 /image 供 web；fb 无相机流，需要编码组件产出 /image
    if cam_type == 'fb':
        # jpeg编码：HbmMsg2160P nv12 -> /image(jpeg ros)，供web展示
        composable_node_descriptions.append(
            ComposableNode(
                package='hobot_codec',
                plugin='HobotCodecNode',
                name='image_jpeg_encoder_node',
                parameters=[
                    {'sub_topic': '/hbmem_img'},
                    {'in_mode': 'shared_mem'},
                    {'in_msg_type': '2160p'},
                    {'in_format': 'nv12'},
                    {'out_mode': 'ros'},
                    {'out_format': 'jpeg'},
                    {'pub_topic': '/image'},
                    {'output_framerate': 5},
                ],
            )
        )

    # web展示节点：组件形式加载（uWS 服务 + nginx 页面仍为独立部署）
    composable_node_descriptions.append(
        ComposableNode(
            package='websocket',
            plugin='websocket::WebsocketComponent',
            name='websocket_node',
            parameters=[
                {'image_topic': '/image'},
                {'image_type': 'mjpeg'},
                {'smart_topic': LaunchConfiguration(
                    "mono_ocr_detection_msg_pub_topic_name")},
                {'only_show_image': False},
                {'output_fps': 0},
                {'channel': 0},
            ],
        )
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=composable_node_descriptions
    )

    # component container
    bringup_cmd_group = GroupAction([
        Node(
            name='tros_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            exec_name='tros_container',
            parameters=[
                {'autostart': 'True'}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen')
    ])

    launch_list = [
        image_source_launch_arg,
        source_fps_launch_arg,
        camera_device_arg,
        camera_fps_launch_arg,
        publish_fps_launch_arg,
        config_file_launch_arg,
        msg_pub_topic_name_launch_arg,
        dump_render_launch_arg,
        declare_container_name_cmd,
        declare_log_level_cmd,
        bringup_cmd_group,
        load_composable_nodes
    ]

    return LaunchDescription(launch_list)
