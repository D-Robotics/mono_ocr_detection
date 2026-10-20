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
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 使能 FastDDS SHM + data_sharing（SYNCHRONOUS 发布 + 零拷贝）：
    # usb 模式下 /image(552KB/帧 mjpeg) 跨进程走 best_effort UDP 会大量丢包
    # （解码有效帧率仅 ~2.5fps），必须启用本环境才能跑满 ~20fps。
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

    # component 形式 4K 检测链，算法输入统一 HbmMsg2160P nv12（/hbmem_img）：
    # usb（默认）：usb_cam(mjpeg /image) -> 解码组件 -> 检测组件 -> web 直通相机 mjpeg
    # fb        ：发布组件(jpg) -> 解码组件 -> 检测组件 -> 编码组件(jpeg /image_jpeg) -> web
    # 注：publisher/codec 使用 loaned message，不能开 intra-process（humble 不支持）
    cam_type = os.getenv('CAM_TYPE', 'usb')
    print("cam_type is ", cam_type)

    # args that can be set from the command line or a default will be used
    config_file_launch_arg = DeclareLaunchArgument(
        "det_config_file",
        default_value=TextSubstitution(text="config/mono_ocr_detection_workconfig.json"))
    dump_render_launch_arg = DeclareLaunchArgument(
        "dump_render_img", default_value=TextSubstitution(text="0"))
    image_width_launch_arg = DeclareLaunchArgument(
        "image_width", default_value=TextSubstitution(text="3840"))
    image_height_launch_arg = DeclareLaunchArgument(
        "image_height", default_value=TextSubstitution(text="2160"))
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='tros_container',
        description='the name of the container that launches all nodes')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='warn', description='log level')

    composable_node_descriptions = []

    if cam_type == 'fb':
        composable_node_descriptions.append(
            ComposableNode(
                package='hobot_image_publisher',
                plugin='PubNode',
                name='image_pub_node',
                parameters=[
                    {"image_source": './config/component.jpg'},
                    {"image_format": 'jpg'},
                    {"msg_pub_topic_name": '/hbmem_img'},
                    {"out_msg_type": '2160p'},
                    {"is_shared_mem": True},
                    {"is_compressed_img_pub": False},
                    {"pub_encoding": 'nv12'},
                    {"fps": 5},
                ],
            ),
        )
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
            ),
        )
    else:
        # usb：usb_cam 组件采集 mjpeg(/image) -> 解码组件（硬解 -> nv12）
        composable_node_descriptions.append(
            ComposableNode(
                package='hobot_usb_cam',
                plugin='usb_cam::HobotUsbCamNode',
                name='hobot_usb_cam_node',
                parameters=[
                    {'image_width': LaunchConfiguration('image_width')},
                    {'image_height': LaunchConfiguration('image_height')},
                    {'framerate': 5},
                    # publish_fps 必须为 0（发布全部采集帧）：该限速实现取
                    # FIFO 最旧帧，非 0 时会引入 ~1.5s 的固定积压延迟
                    {'publish_fps': 0},
                    {'pixel_format': 'mjpeg'},
                    {'io_method': 'mmap'},
                    {'video_device': '/dev/video0'},
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
                    {'in_width': LaunchConfiguration('image_width')},
                    {'in_height': LaunchConfiguration('image_height')},
                    {'out_mode': 'shared_mem'},
                    {'out_msg_type': '2160p'},
                    {'out_format': 'nv12'},
                    {'pub_topic': '/hbmem_img'},
                ],
            ),
        )
    # perception node
    composable_node_descriptions.append(
        ComposableNode(
            package='mono_ocr_detection',
            plugin='MonoOcrDetectionNode',
            name='mono_ocr_detection_node',
            parameters=[
                {'feed_type': 1},
                {'is_shared_mem_sub': 1},
                {'sharedmem_msg_type': '2160p'},
                {'sharedmem_img_topic_name': '/hbmem_img'},
                {'config_file': LaunchConfiguration('det_config_file')},
                {'ai_msg_pub_topic_name': '/hobot_ocr_detection'},
                {'dump_render_img': LaunchConfiguration('dump_render_img')},
                # 与识别节点共享 BPU 时检测推理会被周期性阻塞，帧龄偶发超阈值；
                # 放宽到 2s 让陈旧帧照常处理发布
                {'stale_threshold_ms': 2000},
            ],
            extra_arguments=[{'use_intra_process_comms': False}],
        ),
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=composable_node_descriptions
    )

    # web展示pkg
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_image_type': 'mjpeg',
            'websocket_smart_topic': '/hobot_ocr_detection'
        }.items()
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

    return LaunchDescription([
        config_file_launch_arg,
        dump_render_launch_arg,
        image_width_launch_arg,
        image_height_launch_arg,
        declare_container_name_cmd,
        declare_log_level_cmd,
        # 启动零拷贝容器
        bringup_cmd_group,
        # 加载各组件节点
        load_composable_nodes,
        # 启动web展示pkg
        web_node
    ])
