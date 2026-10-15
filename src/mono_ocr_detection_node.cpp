// Copyright (c) 2026, D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "hobot_cv/hobotcv_imgproc.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "dnn/hb_dnn.h"
#include "include/mono_ocr_detection_node.h"

// Resize an NV12 image preserving aspect ratio so the longest side fits inside
// (scaled_img_width, scaled_img_height). hobot_cv requires output width to be a
// multiple of 16 and height even; we round down and recompute the ratio.
// `ratio` returns the input-to-resized scale, used by PostProcess to map
// detection boxes back to the original frame.
static int ResizeNV12Img(const char *in_img_data,
                  const int &in_img_height,
                  const int &in_img_width,
                  int &resized_img_height,
                  int &resized_img_width,
                  const int &scaled_img_height,
                  const int &scaled_img_width,
                  cv::Mat &out_img,
                  float &ratio) {
  cv::Mat src(in_img_height * 3 / 2, in_img_width, CV_8UC1, (void *)(in_img_data));
  float ratio_w = static_cast<float>(in_img_width) / static_cast<float>(scaled_img_width);
  float ratio_h = static_cast<float>(in_img_height) / static_cast<float>(scaled_img_height);
  float dst_ratio = std::max(ratio_w, ratio_h);
  int resized_width, resized_height;
  if (dst_ratio == ratio_w) {
    resized_width = scaled_img_width;
    // round 而非 int 截断，对齐量化时 letterbox 的 round 逻辑
    resized_height =
        static_cast<int>(std::round(static_cast<float>(in_img_height) / dst_ratio));
  } else if (dst_ratio == ratio_h) {
    resized_width =
        static_cast<int>(std::round(static_cast<float>(in_img_width) / dst_ratio));
    resized_height = scaled_img_height;
  }
  int remain = resized_width % 16;
  if (remain != 0) {
    resized_width -= remain;
    dst_ratio = static_cast<float>(in_img_width) / resized_width;
    resized_height = static_cast<int>(
        std::round(static_cast<float>(in_img_height) / dst_ratio));
  }
  resized_height = resized_height % 2 == 0 ? resized_height : resized_height - 1;
  ratio = dst_ratio;

  resized_img_height = resized_height;
  resized_img_width = resized_width;

  return hobot_cv::hobotcv_resize(
      src, in_img_height, in_img_width, out_img, resized_height, resized_width);
}

static inline double CalTimeMsDuration(const builtin_interfaces::msg::Time &start,
                                       const builtin_interfaces::msg::Time &end) {
  return (static_cast<double>(end.sec) - start.sec) * 1000.0 +
         (static_cast<double>(end.nanosec) - start.nanosec) / 1000000.0;
}
static inline builtin_interfaces::msg::Time ConvertToRosTime(const struct timespec &ts) {
  builtin_interfaces::msg::Time s;
  s.set__sec(ts.tv_sec);
  s.set__nanosec(ts.tv_nsec);
  return s;
}

MonoOcrDetectionNode::MonoOcrDetectionNode(const std::string &node_name, const NodeOptions &options)
    : DnnNode(node_name, options) {
  this->declare_parameter<std::string>("config_file", config_file_);
  this->declare_parameter<int>("stale_threshold_ms", stale_threshold_ms_);
  this->get_parameter<int>("stale_threshold_ms", stale_threshold_ms_);
  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<std::string>("image", image_file_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_pub_topic_name_);
  // 模型选择参数：非空时覆盖 workconfig 里的同名配置（用于免改配置切换模型）
  this->declare_parameter<std::string>("model_file", "");
  this->declare_parameter<std::string>("model_name", "");
  // task_num 覆盖：大分辨率模型（4K）单实例 ION 占用大，需降低并行任务数
  this->declare_parameter<int>("task_num", 0);
  this->declare_parameter<std::string>("sharedmem_img_topic_name",
                                       sharedmem_img_topic_name_);
  this->get_parameter<std::string>("config_file", config_file_);
  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<std::string>("image", image_file_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_pub_topic_name_);

  if (LoadConfig() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Load config fail!");
    rclcpp::shutdown();
    return;
  }
  // OCR 检测只有一个类别 "text"，不需要 vocabulary 文件。
  // 若配置了 vocabulary_file 则加载之，否则用默认类别名。
  LoadVocabulary();
  if (class_names_.empty()) {
    class_names_.push_back("text");
  }
  mono_ocr_detection::SetClassNames(class_names_);

  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Init failed!");
    rclcpp::shutdown();
    return;
  }
  // 注意：NV12 输入模型有 Y+UV 双输入，dnn_node 的 GetModelInputSize 会错误
  // 解析 Y plane 的 shape（width=1）。这里直接用 hbDNN API 从 hbm 查询真实
  // 输入尺寸，查询成功则覆盖 workconfig 的值 —— 切换模型无需改配置。
  {
    int w = 0, h = 0;
    if (QueryModelInputSizeFromHbm(w, h) == 0 &&
        (w != model_input_width_ || h != model_input_height_)) {
      RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection"),
                  "Model input size auto-detected: %dx%d (config had %dx%d, override)",
                  w, h, model_input_width_, model_input_height_);
      model_input_width_ = w;
      model_input_height_ = h;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection"),
                  "Model input size: %d x %d", model_input_width_,
                  model_input_height_);
    }
  }

  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      ai_msg_pub_topic_name_, 10);

  if (feed_type_ == 0) {
    RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection"), "Dnn node feed with local image: %s",
                image_file_.c_str());
    FeedFromLocal();
  } else if (feed_type_ == 1) {
    if (is_shared_mem_sub_) {
#ifdef SHARED_MEM_ENABLED
      sharedmem_img_subscription_4k_ =
          this->create_subscription<hbm_img_msgs::msg::HbmMsg2160P>(
              sharedmem_img_topic_name_, rclcpp::SensorDataQoS(),
              std::bind(&MonoOcrDetectionNode::SharedMemImgProcess, this,
                        std::placeholders::_1));
      RCLCPP_WARN(rclcpp::get_logger("mono_ocr_detection"),
                  "Subscribe 4K shared mem img: %s (HbmMsg2160P)",
                  sharedmem_img_topic_name_.c_str());
#else
      RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Unsupport shared mem");
#endif
    } else {
      ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          ros_img_sub_topic_name_, 10,
          std::bind(&MonoOcrDetectionNode::RosImgProcess, this, std::placeholders::_1));
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Invalid feed_type:%d", feed_type_);
    rclcpp::shutdown();
  }
}

MonoOcrDetectionNode::~MonoOcrDetectionNode() {}

int MonoOcrDetectionNode::LoadConfig() {
  if (config_file_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Config file is empty!");
    return -1;
  }
  std::ifstream ifs(config_file_.c_str());
  if (!ifs) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Read config file [%s] fail!",
                 config_file_.c_str());
    return -1;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError()) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Parsing config file %s failed",
                 config_file_.c_str());
    return -1;
  }
  if (document.HasMember("model_file")) model_file_name_ = document["model_file"].GetString();
  if (document.HasMember("vocabulary_file")) vocabulary_file_name_ = document["vocabulary_file"].GetString();
  if (document.HasMember("model_name")) model_name_ = document["model_name"].GetString();
  if (document.HasMember("task_num")) task_num_ = document["task_num"].GetInt();
  if (document.HasMember("class_num")) class_num_ = document["class_num"].GetInt();
  if (document.HasMember("model_input_width")) model_input_width_ = document["model_input_width"].GetInt();
  if (document.HasMember("model_input_height")) model_input_height_ = document["model_input_height"].GetInt();

  // ros param 覆盖 workconfig（免改配置文件切换模型）
  std::string param_model_file, param_model_name;
  this->get_parameter<std::string>("model_file", param_model_file);
  this->get_parameter<std::string>("model_name", param_model_name);
  if (!param_model_file.empty()) model_file_name_ = param_model_file;
  if (!param_model_name.empty()) model_name_ = param_model_name;
  int param_task_num = 0;
  this->get_parameter<int>("task_num", param_task_num);
  if (param_task_num > 0) task_num_ = param_task_num;
  this->get_parameter<std::string>("sharedmem_img_topic_name",
                                   sharedmem_img_topic_name_);
  RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection"), "Model file: %s, name: %s",
              model_file_name_.c_str(), model_name_.c_str());

  if (mono_ocr_detection::LoadConfig(document) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Load parser config from [%s] fail!",
                 config_file_.c_str());
    return -1;
  }
  return 0;
}

int MonoOcrDetectionNode::QueryModelInputSizeFromHbm(int &w, int &h) {
  // 直接用 hbDNN 原生 API 查询 NV12 双输入模型的 Y plane 形状：
  // x_y validShape = [1, H, W, 1]（dnn_node 的 GetModelInputSize 按 NCHW
  // 解析 NHWC 会得到 width=1，这里绕开它）。
  hbDNNPackedHandle_t packed_dnn = nullptr;
  hbDNNHandle_t dnn_handle = nullptr;
  const char *model_file = model_file_name_.c_str();
  int ret = hbDNNInitializeFromFiles(&packed_dnn, &model_file, 1);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"),
                 "hbDNNInitializeFromFiles(%s) failed: %d", model_file, ret);
    return -1;
  }
  const char *name = model_name_.empty() ? nullptr : model_name_.c_str();
  ret = hbDNNGetModelHandle(&dnn_handle, packed_dnn, name);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"),
                 "hbDNNGetModelHandle(%s) failed: %d", name, ret);
    hbDNNRelease(packed_dnn);
    return -1;
  }
  int32_t input_count = 0;
  hbDNNGetInputCount(&input_count, dnn_handle);
  if (input_count < 1) {
    hbDNNRelease(packed_dnn);
    return -1;
  }
  hbDNNTensorProperties props;
  ret = hbDNNGetInputTensorProperties(&props, dnn_handle, 0);
  if (ret != 0 || props.validShape.numDimensions < 4) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"),
                 "hbDNNGetInputTensorProperties failed: %d", ret);
    hbDNNRelease(packed_dnn);
    return -1;
  }
  // NV12 模型输入 0 = Y plane，NHWC [1, H, W, 1]，U8；
  // featuremap 模型输入 0 = 逻辑 x，NCHW [1, 3, H, W]，F32。
  if (props.tensorType == HB_DNN_TENSOR_TYPE_U8) {
    h = props.validShape.dimensionSize[1];
    w = props.validShape.dimensionSize[2];
  } else {
    h = props.validShape.dimensionSize[2];
    w = props.validShape.dimensionSize[3];
  }
  hbDNNRelease(packed_dnn);
  if (w <= 0 || h <= 0) return -1;
  return 0;
}

int MonoOcrDetectionNode::LoadVocabulary() {
  if (vocabulary_file_name_.empty()) return 0;  // allow no vocabulary
  std::ifstream ifs(vocabulary_file_name_.c_str());
  if (!ifs) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Read vocabulary fail: %s",
                 vocabulary_file_name_.c_str());
    return -1;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document doc;
  doc.ParseStream(isw);
  if (doc.HasParseError() || !doc.IsArray()) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Vocabulary format error");
    return -1;
  }
  class_names_.clear();
  for (const auto &item : doc.GetArray()) {
    if (item.IsArray()) {
      for (const auto &si : item.GetArray()) {
        if (si.IsString()) { class_names_.emplace_back(si.GetString()); break; }
      }
    }
    if (item.IsString()) class_names_.emplace_back(item.GetString());
  }
  return 0;
}

int MonoOcrDetectionNode::SetNodePara() {
  if (!dnn_node_para_ptr_) return -1;
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = hobot::dnn_node::ModelTaskType::ModelInferType;
  dnn_node_para_ptr_->task_num = task_num_;
  return 0;
}

int MonoOcrDetectionNode::FeedFromLocal() {
  if (access(image_file_.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Image: %s not exist!", image_file_.c_str());
    return -1;
  }
  auto dnn_output = std::make_shared<MonoOcrDetectionOutput>();
  struct timespec ts = {0, 0};
  clock_gettime(CLOCK_REALTIME, &ts);
  dnn_output->perf_preprocess.stamp_start.sec = ts.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = ts.tv_nsec;

  RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection"),
              "before GetNV12PyramidFromBGR, model=%dx%d",
              model_input_height_, model_input_width_);
  auto pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromBGR(
      image_file_, dnn_output->img_h, dnn_output->img_w,
      dnn_output->resized_h, dnn_output->resized_w,
      model_input_height_, model_input_width_);
  RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection"),
              "after GetNV12PyramidFromBGR, pyramid=%s, img=%dx%d",
              pyramid ? "ok" : "null", dnn_output->img_w, dnn_output->img_h);
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Get NV12 pyramid from %s failed!",
                 image_file_.c_str());
    return -1;
  }
  dnn_output->model_h = model_input_height_;
  dnn_output->model_w = model_input_width_;

  clock_gettime(CLOCK_REALTIME, &ts);
  dnn_output->perf_preprocess.stamp_end.sec = ts.tv_sec;
  dnn_output->perf_preprocess.stamp_end.nanosec = ts.tv_nsec;

  dnn_output->pyramid = pyramid;
  dnn_output->msg_header.set__frame_id("feedback");
  dnn_output->msg_header.set__stamp(rclcpp::Clock().now());

  RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection"),
              "before Run, pyramid h=%d w=%d y_stride=%d uv_stride=%d",
              pyramid->height, pyramid->width, pyramid->y_stride,
              pyramid->uv_stride);
  std::vector<std::shared_ptr<DNNInput>> inputs = {pyramid};
  int ret = Run(inputs, dnn_output, nullptr, true);
  RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection"),
              "after Run, ret=%d", ret);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Run failed, ret=%d", ret);
    return -1;
  }
  return 0;
}

void MonoOcrDetectionNode::RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (!rclcpp::ok() || !msg) return;
  auto dnn_output = std::make_shared<MonoOcrDetectionOutput>();
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;

  if ("rgb8" == msg->encoding || "bgr8" == msg->encoding) {
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    dnn_output->resized_h = model_input_height_;
    dnn_output->resized_w = model_input_width_;
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromBGRImg(
        cv_ptr->image, model_input_height_, model_input_width_);
  } else if ("nv12" == msg->encoding || "nv12_8uc1" == msg->encoding) {
    if (msg->height != static_cast<uint32_t>(model_input_height_) ||
        msg->width != static_cast<uint32_t>(model_input_width_)) {
      cv::Mat out_img;
      if (ResizeNV12Img(reinterpret_cast<const char *>(msg->data.data()),
                        msg->height, msg->width,
                        dnn_output->resized_h, dnn_output->resized_w,
                        model_input_height_, model_input_width_,
                        out_img, dnn_output->ratio) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Resize nv12 img fail!");
        return;
      }
      uint32_t out_img_width = out_img.cols;
      uint32_t out_img_height = out_img.rows * 2 / 3;
      pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
          reinterpret_cast<const char *>(out_img.data),
          out_img_height, out_img_width, model_input_height_, model_input_width_);
    } else {
      dnn_output->resized_h = std::min(static_cast<int>(msg->height), model_input_height_);
      dnn_output->resized_w = std::min(static_cast<int>(msg->width), model_input_width_);
      pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
          reinterpret_cast<const char*>(msg->data.data()), msg->height, msg->width,
          model_input_height_, model_input_width_);
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Unsupported img encoding: %s",
                 msg->encoding.c_str());
    return;
  }
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Get NV12 pyramid failed!");
    return;
  }

  dnn_output->img_h = msg->height;
  dnn_output->img_w = msg->width;
  dnn_output->model_h = model_input_height_;
  dnn_output->model_w = model_input_width_;
  dnn_output->pyramid = pyramid;
  dnn_output->msg_header = msg->header;

  std::vector<std::shared_ptr<DNNInput>> inputs = {pyramid};
  int ret = Run(inputs, dnn_output, nullptr, false);
  if (ret == HB_DNN_TASK_NUM_EXCEED_LIMIT) return;
  if (ret != 0) RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Run failed, ret=%d", ret);
}

#ifdef SHARED_MEM_ENABLED
void MonoOcrDetectionNode::SharedMemImgProcess(const hbm_img_msgs::msg::HbmMsg2160P::ConstSharedPtr msg) {
  if (!rclcpp::ok() || !msg) return;

  std::stringstream ss;
  ss << "Recved img encoding: "
     << std::string(reinterpret_cast<const char *>(msg->encoding.data()))
     << ", h: " << msg->height << ", w: " << msg->width
     << ", step: " << msg->step << ", index: " << msg->index
     << ", stamp: " << msg->time_stamp.sec << "_"
     << msg->time_stamp.nanosec << ", data size: " << msg->data_size;
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  
  ProcessSharedMemNv12(reinterpret_cast<const uint8_t *>(msg->data.data()),
                       msg->height, msg->width,
                       std::string(reinterpret_cast<const char *>(msg->encoding.data())),
                       msg->time_stamp);
}

void MonoOcrDetectionNode::ProcessSharedMemNv12(const uint8_t *data,
                                                uint32_t msg_h,
                                                uint32_t msg_w,
                                                const std::string &encoding,
                                                const builtin_interfaces::msg::Time &stamp) {
  // // 丢旧帧：det 4K 推理 ~210ms/帧跟不上输入速率时，hbmem 缓冲池会积压到
  // // 池深（实测可达 ~3s），导致 1) 识别结果时间戳严重滞后；2) websocket 按
  // // stamp 配对失败（±600ms 窗口）web 黑屏。只处理 400ms 内的新鲜帧。
  // {
  //   const auto now = std::chrono::system_clock::now();
  //   const auto age = now - std::chrono::system_clock::time_point(
  //       std::chrono::seconds(stamp.sec) +
  //       std::chrono::nanoseconds(stamp.nanosec));
  //   const auto age_ms =
  //       std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
  //   if (age_ms > stale_threshold_ms_) {
  //     static int drop_cnt = 0;
  //     if (++drop_cnt % 30 == 1) {
  //       RCLCPP_WARN(rclcpp::get_logger("mono_ocr_detection"),
  //                   "drop stale frame: age %ld ms (dropped %d)", age_ms, drop_cnt);
  //     }
  //     return;
  //   }
  // }
  if (!rclcpp::ok()) return;
  auto dnn_output = std::make_shared<MonoOcrDetectionOutput>();
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;

  if ("nv12" == encoding) {
    // 不做 CPU 预缩放：原始帧直接建 pyramid，由 BPU NV12 预处理一次拉伸到
    // 模型输入。双重重采样会模糊小字（4K 模型小标注漏检的根因），且省 CPU。
    // parser 输出的框在模型空间，resized 需置为模型输入尺寸，PostProcess 才会
    // 按 img/model 的比值把框回投到原图分辨率（resized=msg 会导致比例恒 1.0，
    // web 叠加时框整体偏移）。
    dnn_output->resized_h = model_input_height_;
    dnn_output->resized_w = model_input_width_;
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(data), msg_h, msg_w,
        model_input_height_, model_input_width_);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"),
                 "Unsupported img encoding: %s, only nv12 for shared mem.",
                 encoding.c_str());
    return;
  }
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Get NV12 pyramid from shared mem failed!");
    return;
  }

  dnn_output->img_h = msg_h;
  dnn_output->img_w = msg_w;
  dnn_output->model_h = model_input_height_;
  dnn_output->model_w = model_input_width_;
  dnn_output->pyramid = pyramid;
  // websocket 按 smart/image 的 header.stamp 精确配对推送，时间戳必须透传
  dnn_output->msg_header.stamp = stamp;
  dnn_output->msg_header.frame_id = "default_cam";

  std::vector<std::shared_ptr<DNNInput>> inputs = {pyramid};
  int ret = Run(inputs, dnn_output, nullptr, false);
  if (ret == HB_DNN_TASK_NUM_EXCEED_LIMIT) return;
  if (ret != 0) RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Run failed, ret=%d", ret);
}
#endif

int MonoOcrDetectionNode::PostProcess(const std::shared_ptr<DnnNodeOutput> &node_output) {
  if (!rclcpp::ok()) return -1;
  struct timespec ts = {0, 0};
  clock_gettime(CLOCK_REALTIME, &ts);
  auto pout = std::dynamic_pointer_cast<MonoOcrDetectionOutput>(node_output);
  if (node_output->output_tensors.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Invalid output_tensors");
    return -1;
  }

  auto det_result = std::make_shared<DnnParserResult>();
  if (mono_ocr_detection::Parse(node_output, det_result) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection"), "Parse failed");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection"),
              "PostProcess detected %zu text boxes",
              det_result->perception.det.size());
  if (!msg_publisher_) return -1;

  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(new ai_msgs::msg::PerceptionTargets());

  for (auto &rect : det_result->perception.det) {
    if (rect.bbox.xmin < 0) rect.bbox.xmin = 0;
    if (rect.bbox.ymin < 0) rect.bbox.ymin = 0;
    if (rect.bbox.xmax >= model_input_width_) rect.bbox.xmax = model_input_width_ - 1;
    if (rect.bbox.ymax >= model_input_height_) rect.bbox.ymax = model_input_height_ - 1;

    ai_msgs::msg::Roi roi;
    roi.set__type(rect.class_name);
    roi.rect.set__x_offset(rect.bbox.xmin);
    roi.rect.set__y_offset(rect.bbox.ymin);
    roi.rect.set__width(rect.bbox.xmax - rect.bbox.xmin);
    roi.rect.set__height(rect.bbox.ymax - rect.bbox.ymin);
    roi.set__confidence(rect.score);
    ai_msgs::msg::Target target;
    target.set__type(rect.class_name);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }

  pub_data->header.set__stamp(pout->msg_header.stamp);
  pub_data->header.set__frame_id(pout->msg_header.frame_id);

  if (dump_render_img_ && pout->pyramid) {
    ImageUtils::Render(pout->pyramid, pub_data, pout->resized_h, pout->resized_w, ".");
  }

  // 坐标映射：模型坐标(736x1536 letterbox) -> 原图坐标。
  // 注意 x/y 需分别映射：x 方向 resize 填满(ratio_x)，y 方向 resize 后底部有
  // padding(ratio_y)，两者略有差异。用 resized_h/w 精确计算，而非统一 ratio。
  if (pout->img_w > 0 && pout->img_h > 0 &&
      pout->resized_w > 0 && pout->resized_h > 0) {
    float ratio_x = static_cast<float>(pout->img_w) / pout->resized_w;
    float ratio_y = static_cast<float>(pout->img_h) / pout->resized_h;
    for (auto &target : pub_data->targets) {
      for (auto &roi : target.rois) {
        roi.rect.x_offset = static_cast<uint32_t>(roi.rect.x_offset * ratio_x);
        roi.rect.y_offset = static_cast<uint32_t>(roi.rect.y_offset * ratio_y);
        roi.rect.width = static_cast<uint32_t>(roi.rect.width * ratio_x);
        roi.rect.height = static_cast<uint32_t>(roi.rect.height * ratio_y);
        if (roi.rect.x_offset >= pout->img_w) roi.rect.x_offset = pout->img_w - 1;
        if (roi.rect.y_offset >= pout->img_h) roi.rect.y_offset = pout->img_h - 1;
        if (roi.rect.x_offset + roi.rect.width >= pout->img_w)
          roi.rect.width = pout->img_w - 1 - roi.rect.x_offset;
        if (roi.rect.y_offset + roi.rect.height >= pout->img_h)
          roi.rect.height = pout->img_h - 1 - roi.rect.y_offset;
      }
    }
  }

  if (node_output->rt_stat) {
    ai_msgs::msg::Perf perf;
    perf.set__type("mono_ocr_detection_infer");
    perf.stamp_start = ConvertToRosTime(node_output->rt_stat->infer_timespec_start);
    perf.stamp_end = ConvertToRosTime(node_output->rt_stat->infer_timespec_end);
    perf.set__time_ms_duration(node_output->rt_stat->infer_time_ms);
    pub_data->perfs.push_back(perf);

    ai_msgs::msg::Perf perf_post;
    perf_post.set__type("mono_ocr_detection_postprocess");
    perf_post.stamp_start = ConvertToRosTime(ts);
    struct timespec ts2 = {0, 0};
    clock_gettime(CLOCK_REALTIME, &ts2);
    perf_post.stamp_end = ConvertToRosTime(ts2);
    perf_post.set__time_ms_duration(CalTimeMsDuration(perf_post.stamp_start, perf_post.stamp_end));
    pub_data->perfs.emplace_back(perf_post);
  }

  // 手动统计 fps（每 2 秒刷新一次）
  {
    auto now = std::chrono::steady_clock::now();
    if (frame_count_ == 0) {
      fps_start_ = now;
    }
    frame_count_++;
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now - fps_start_).count();
    if (elapsed_ms >= 2000) {
      output_fps_ =
          static_cast<int>(frame_count_ * 1000.0f / elapsed_ms + 0.5f);
      frame_count_ = 0;
      fps_start_ = now;
    }
  }
  pub_data->set__fps(output_fps_);

  msg_publisher_->publish(std::move(pub_data));
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(MonoOcrDetectionNode)
