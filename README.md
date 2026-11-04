# mono_ocr_detection

English | [简体中文](README_cn.md)

mono_ocr_detection package, [PaddleOCR](https://github.com/PaddlePaddle/PaddleOCR) algorithm quanti model on RDK platform based on dnn_node.

## Configuration

`config/mono_ocr_detection_workconfig.json` (edit to match your model / post-process):

```json
{
    "model_file": "config/ppocrv6_medium_det_int8_640_m_m.hbm",
    "task_num": 1,
    "class_num": 1,
    "thresh": 0.2,
    "box_thresh": 0.2,
    "max_candidates": 1000,
    "unclip_ratio": 1.5,
    "min_size": 3.0,
    "nms_iou": 0.5
}

```

## Build

RDK X5: `bash robot_dev_config/build.sh -p X5 -s mono_ocr_detection`.

RDK S100: `bash robot_dev_config/build.sh -p S100 -s mono_ocr_detection`.

## Run

- Single-image feedback (no launch)

```shell
ros2 run mono_ocr_detection mono_ocr_detection --ros-args -p feed_type:=0 -p config_file:=config/mono_ocr_detection_workconfig.json -p dump_render_img:=1
```

- Camera mode (mipi)

```shell
export CAM_TYPE=mipi
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch mono_ocr_detection mono_ocr_detection.launch.py
```

- Camera mode (usb)

```shell
export CAM_TYPE=usb
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch mono_ocr_detection mono_ocr_detection.launch.py
```

- Image feedback mode

```shell
export CAM_TYPE=fb
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch mono_ocr_detection mono_ocr_detection.launch.py mono_ocr_detection_config_file:=config/mono_ocr_detection_workconfig.json
```

- Component 4K pipeline (recommended, one launch for live usb / feedback fb)

```shell
source /opt/ros/humble/setup.bash
source ./install/setup.bash

# usb live (default): starts usb_cam + decode + OCR + websocket
ros2 launch mono_ocr_detection mono_ocr_detection_component.launch.py device:=/dev/video0

# fb feedback: local 4K jpg -> HbmMsg2160P -> OCR -> encode -> web
export CAM_TYPE=fb
ros2 launch mono_ocr_detection mono_ocr_detection_component.launch.py image_source:=./config/component.jpg
```

The pipeline uses `HbmMsg2160P` (3840x2160 NV12) shared-memory messages; all
nodes run in one process (`tros_container`) and exchange frames over FastDDS
SHM zero-copy.

## Node ROS parameters

| Parameter | Default | Description |
|---|---|---|
| `config_file` | `config/mono_ocr_detection_workconfig.json` | JSON config file |
| `feed_type` | `0` | 0: local image, 1: subscribe image |
| `resolution` | `1080P` | share mem msg type: `1080P` / `2160P`  |
| `image` | `config/component.jpg` | feedback image path |
| `dump_render_img` | `0` | save rendered image |
| `is_shared_mem_sub` | `0` | subscribe `/hbmem_img` via shared memory |
| `ai_msg_pub_topic_name` | `/hobot_ocr_detection` | AI message topic |
