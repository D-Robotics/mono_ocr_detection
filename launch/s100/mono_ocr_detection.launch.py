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
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    # 拷贝算法config到cwd，节点里的 config/xxx 相对路径才能解析
    mono_ocr_path = os.path.join(
        get_package_prefix('mono_ocr_detection'), "lib/mono_ocr_detection")
    print("mono_ocr_path is ", mono_ocr_path)
    os.system("cp -rn " + mono_ocr_path + "/config/. ./config/ 2>/dev/null")

    config_file_launch_arg = DeclareLaunchArgument(
        "mono_ocr_detection_config_file", default_value=TextSubstitution(text="config/mono_ocr_detection_workconfig.json")
    )
    image_width_launch_arg = DeclareLaunchArgument(
        "mono_ocr_detection_image_width", default_value=TextSubstitution(text="3840")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "mono_ocr_detection_image_height", default_value=TextSubstitution(text="2160")
    )
    dump_render_launch_arg = DeclareLaunchArgument(
        "mono_ocr_detection_dump_render_img", default_value=TextSubstitution(text="0")
    )
    msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "mono_ocr_detection_msg_pub_topic_name", default_value=TextSubstitution(text="/hobot_ocr_detection")
    )
    camera_type = os.getenv('CAM_TYPE')
    print("camera_type is ", camera_type)

    cam_node = None
    camera_type_mipi = None
    camera_device_arg = None

    if camera_type == "usb":
        usb_cam_device_arg = DeclareLaunchArgument(
            'device', default_value='/dev/video0', description='usb camera device')
        usb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('hobot_usb_cam'),
                             'launch/hobot_usb_cam.launch.py')),
            launch_arguments={
                'usb_image_width': LaunchConfiguration('mono_ocr_detection_image_width'),
                'usb_image_height': LaunchConfiguration('mono_ocr_detection_image_height'),
                'usb_framerate': '30',
                'usb_video_device': LaunchConfiguration('device')
            }.items()
        )
        cam_node = usb_node
        camera_type_mipi = False
        camera_device_arg = usb_cam_device_arg

    elif camera_type == "fb":
        feedback_picture_arg = DeclareLaunchArgument(
            'publish_image_source', default_value='./config/component.jpg',
            description='feedback picture')
        fb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('hobot_image_publisher'),
                             'launch/hobot_image_publisher.launch.py')),
            launch_arguments={
                'publish_image_source': LaunchConfiguration('publish_image_source'),
                'publish_message_topic_name': '/hbmem_img',
                'publish_image_format': 'jpg',
                'publish_out_msg_type': '2160p',
                'publish_is_shared_mem': 'True',
                'publish_is_compressed_img_pub': 'False',
                'publish_fps': '5'
            }.items()
        )
        cam_node = fb_node
        camera_type_mipi = True
        camera_device_arg = feedback_picture_arg

    else:
        if camera_type == "mipi":
            print("using mipi cam")
        else:
            print("invalid camera_type ", camera_type, ", using default mipi cam")
        mipi_cam_device_arg = DeclareLaunchArgument(
            'device', default_value='F37', description='mipi camera device')
        mipi_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('mipi_cam'),
                             'launch/mipi_cam.launch.py')),
            launch_arguments={
                'mipi_image_width': LaunchConfiguration('mono_ocr_detection_image_width'),
                'mipi_image_height': LaunchConfiguration('mono_ocr_detection_image_height'),
                'mipi_io_method': 'shared_mem',
                'mipi_frame_ts_type': 'realtime',
                'mipi_video_device': LaunchConfiguration('device')
            }.items()
        )
        cam_node = mipi_node
        camera_type_mipi = True
        camera_device_arg = mipi_cam_device_arg

    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hobot_codec'),
                         'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'shared_mem', 
            'codec_out_mode': 'ros',
            'codec_in_msg_type': '2160p', 
            'codec_in_format': 'nv12',
            'codec_out_format': 'jpeg',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image',
            'codec_output_framerate': '5'
        }.items()
    )

    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hobot_codec'),
                         'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_in_width': '3840',
            'codec_in_height': '2160',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image',
            'codec_out_format': 'nv12',
            # usb 为 4K 相机：解码输出走 2160P 档（HbmMsg2160P），
            # 默认 1080p 档缓冲只有 6.2MB，4K NV12(12.4MB) 会溢出
            'codec_out_msg_type': '2160p',
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )

    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('websocket'),
                         'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_image_type': 'mjpeg',
            'websocket_smart_topic': LaunchConfiguration('mono_ocr_detection_msg_pub_topic_name')
        }.items()
    )

    node = Node(
        package='mono_ocr_detection', executable='mono_ocr_detection', output='screen',
        parameters=[
            {"config_file": LaunchConfiguration("mono_ocr_detection_config_file")},
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {"dump_render_img": LaunchConfiguration("mono_ocr_detection_dump_render_img")},
            {"ai_msg_pub_topic_name": LaunchConfiguration("mono_ocr_detection_msg_pub_topic_name")},
            {'stale_threshold_ms': 2000},
            # 算法固定订阅 HbmMsg2160P 全分辨率帧，话题与图片源对齐
            {"sharedmem_img_topic_name": "/hbmem_img"}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hobot_shm'),
                         'launch/hobot_shm.launch.py'))
    )

    if camera_type_mipi:
        return LaunchDescription([
            camera_device_arg, 
            config_file_launch_arg,
            image_width_launch_arg,
            image_height_launch_arg,
            dump_render_launch_arg,
            msg_pub_topic_name_launch_arg,
            shared_mem_node,
            cam_node,
            jpeg_codec_node,
            node,
            web_node
        ])
    else:
        return LaunchDescription([
            camera_device_arg,
            config_file_launch_arg,
            image_width_launch_arg,
            image_height_launch_arg,
            dump_render_launch_arg,
            msg_pub_topic_name_launch_arg,
            shared_mem_node,
            cam_node,
            nv12_codec_node,
            node,
            web_node
        ])
