#ifndef MONO_OCR_DETECTION_NODE_H_
#define MONO_OCR_DETECTION_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "hbm_img_msgs/msg/hbm_msg2160_p.hpp"
#endif

#include "ai_msgs/msg/perception_targets.hpp"
#include "ai_msgs/msg/perf.hpp"
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"

#include "include/image_utils.h"
#include "include/post_process/mono_ocr_detection_output_parser.h"

using rclcpp::NodeOptions;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DNNInput;
using hobot::dnn_node::NV12PyramidInput;
using hobot::dnn_node::output_parser::DnnParserResult;

struct MonoOcrDetectionOutput : public DnnNodeOutput {
  float ratio = 1.0;
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid;
  ai_msgs::msg::Perf perf_preprocess;
  int model_h = 0, model_w = 0;
  int img_h = 0, img_w = 0;
  int resized_h = 0, resized_w = 0;
  std_msgs::msg::Header msg_header;
};

class MonoOcrDetectionNode : public DnnNode {
 public:
  MonoOcrDetectionNode(const std::string &node_name,
          const NodeOptions &options = NodeOptions());
  // component 容器加载入口
  explicit MonoOcrDetectionNode(const rclcpp::NodeOptions &options)
      : MonoOcrDetectionNode("mono_ocr_detection", options) {}
  ~MonoOcrDetectionNode() override;

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  int LoadConfig();
  int LoadVocabulary();

  // 从 hbm 模型直接查询 NV12 输入的 H/W（Y plane validShape [1,H,W,1]），
  // 用于模型自动适配（切换模型无需改 workconfig 的 model_input_width/height）
  int QueryModelInputSizeFromHbm(int &w, int &h);

  int FeedFromLocal();

#ifdef SHARED_MEM_ENABLED
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg2160P::ConstSharedPtr msg);
  // 两种共享内存消息的公共处理（字段同构：encoding/width/height/data）
  // stamp 用于输出消息头：websocket 按 smart/image 时间戳配对推送，必须透传
  void ProcessSharedMemNv12(const uint8_t *data, uint32_t height,
                            uint32_t width, const std::string &encoding,
                            const builtin_interfaces::msg::Time &stamp);
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg2160P>::ConstSharedPtr
      sharedmem_img_subscription_4k_ = nullptr;
#endif

  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;

  std::string config_file_ = "config/mono_ocr_detection_workconfig.json";
  std::string model_file_name_ = "";
  std::string model_name_ = "";
  std::string vocabulary_file_name_ = "";
  std::string image_file_ = "config/component.jpg";
  int feed_type_ = 0;
  // 陈旧帧丢弃阈值 ms（参数 stale_threshold_ms）
  int stale_threshold_ms_ = 400;
  int is_shared_mem_sub_ = 0;
  int dump_render_img_ = 0;
  int task_num_ = 4;

  int model_input_width_ = 1536;
  int model_input_height_ = 736;

  int class_num_ = 1;
  std::vector<std::string> class_names_;

  std::string ai_msg_pub_topic_name_ = "/hobot_ocr_detection";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr
      msg_publisher_ = nullptr;

  std::string sharedmem_img_topic_name_ = "/hbmem_img";
  std::string ros_img_sub_topic_name_ = "/image";

  // 手动 fps 统计（rt_stat->output_fps 可能未更新）
  int frame_count_ = 0;
  int output_fps_ = 0;
  std::chrono::steady_clock::time_point fps_start_;
};

#endif  // MONO_OCR_DETECTION_NODE_H_
