# mono_ocr_detection

[English](README.md) | 简体中文

基于 dnn_node 的 mono_ocr_detection 算法节点。JSON 配置驱动。

## 配置

`config/mono_ocr_detection_workconfig.json`（按实际模型 / 后处理修改）：

```json
{
    "model_file": "config/ppocrv6_medium_det_int8_4k_m.hbm",
    "model_name": "ppocrv6_medium_det_int8_4k_m",
    "task_num": 1,
    "class_num": 1,
    "model_input_width": 3840,
    "model_input_height": 2176,
    "thresh": 0.2,
    "box_thresh": 0.2,
    "max_candidates": 1000,
    "unclip_ratio": 1.5,
    "min_size": 3.0,
    "nms_iou": 0.5
}
```

> TODO: `model_output_count / reg_max / strides / score_threshold /
> nms_threshold / nms_top_k / output_order` 等占位值需替换为算法真实值。

## 编译

RDK S100：
`bash robot_dev_config/build.sh -p S100 -s mono_ocr_detection`。

## 运行

- 单张图片回灌（无需 launch）

```shell
ros2 run mono_ocr_detection mono_ocr_detection --ros-args -p feed_type:=0 -p config_file:=config/mono_ocr_detection_workconfig.json -p dump_render_img:=1
```

- 相机模式（mipi）

```shell
export CAM_TYPE=mipi
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch mono_ocr_detection mono_ocr_detection.launch.py
```

- 相机模式（usb）

```shell
export CAM_TYPE=usb
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch mono_ocr_detection mono_ocr_detection.launch.py
```

- 图片回灌模式

```shell
export CAM_TYPE=fb
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch mono_ocr_detection mono_ocr_detection.launch.py mono_ocr_detection_config_file:=config/mono_ocr_detection_workconfig.json
```

- component 形式 4K 全链（推荐，usb 实时 / fb 回灌同一条 launch）

```shell
source /opt/ros/humble/setup.bash
source ./install/setup.bash

# usb 实时（默认）：usb_cam + 解码 + OCR + websocket 全部拉起
ros2 launch mono_ocr_detection mono_ocr_detection_component.launch.py device:=/dev/video0

# fb 回灌：本地 4K jpg -> HbmMsg2160P -> OCR -> 编码 -> web
export CAM_TYPE=fb
ros2 launch mono_ocr_detection mono_ocr_detection_component.launch.py image_source:=./config/component.jpg
```

链路统一使用 `HbmMsg2160P`（3840×2160 NV12）共享内存消息，各节点同进程
（`tros_container`）运行，消息走 FastDDS SHM 零拷贝。

## 节点 ROS 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `config_file` | `config/mono_ocr_detection_workconfig.json` | JSON 配置文件 |
| `feed_type` | `0` | 0: 本地图片，1: 订阅图片 |
| `image` | `config/component.jpg` | 回灌图片路径 |
| `dump_render_img` | `0` | 保存渲染结果图 |
| `is_shared_mem_sub` | `0` | 通过共享内存订阅 `/hbmem_img` |
| `ai_msg_pub_topic_name` | `/hobot_ocr_detection` | AI 消息发布话题 |
