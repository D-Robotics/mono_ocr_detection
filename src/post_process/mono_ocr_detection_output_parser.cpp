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

#include "include/post_process/mono_ocr_detection_output_parser.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

#include "rapidjson/document.h"
#include "include/post_process/local_nms.h"
#include "rclcpp/rclcpp.hpp"

#include "dnn_node/util/output_parser/utils.h"
#include "opencv2/imgproc.hpp"

namespace mono_ocr_detection {

// DB 后处理参数（对齐 PaddleOCR configs/det/PP-OCRv6/PP-OCRv6_medium_det.yml）
struct ParserConfig {
  int class_num;
  std::vector<std::string> class_names;
  std::vector<int> output_order;
  // DB post-process params
  float thresh;        // 二值化阈值 (DBPostProcess.thresh)
  float box_thresh;    // 文本框置信度阈值 (DBPostProcess.box_thresh)
  int max_candidates;  // 最大候选框数
  float unclip_ratio;  // 文本框膨胀比例 (DBPostProcess.unclip_ratio)
  float min_size;      // 最小文本框边长
  float nms_iou;       // 重叠框 NMS IoU 阈值（<=0 关闭）
};

static ParserConfig g_config = {
    1, {}, {}, 0.3f, 0.6f, 1000, 1.5f, 3.0f, 0.5f
};

void SetClassNames(const std::vector<std::string> &names) {
  if (names.empty()) return;
  g_config.class_names = names;
}

int LoadConfig(const rapidjson::Document &document) {
  if (document.HasMember("class_num"))
    g_config.class_num = document["class_num"].GetInt();
  if (document.HasMember("thresh"))
    g_config.thresh = document["thresh"].GetFloat();
  if (document.HasMember("box_thresh"))
    g_config.box_thresh = document["box_thresh"].GetFloat();
  if (document.HasMember("max_candidates"))
    g_config.max_candidates = document["max_candidates"].GetInt();
  if (document.HasMember("unclip_ratio"))
    g_config.unclip_ratio = document["unclip_ratio"].GetFloat();
  if (document.HasMember("nms_iou"))
    g_config.nms_iou = document["nms_iou"].GetFloat();
  if (document.HasMember("min_size"))
    g_config.min_size = document["min_size"].GetFloat();
  return 0;
}

static void SortByOrder(
    std::vector<std::shared_ptr<hobot::dnn_node::DNNTensor>> &outputs,
    std::vector<int> order) {
  std::vector<std::shared_ptr<hobot::dnn_node::DNNTensor>> sorted(
      outputs.size());
  for (size_t i = 0; i < outputs.size(); i++) sorted[i] = outputs[order[i]];
  outputs = sorted;
}

// 对文本框做 unclip 膨胀（近似 PaddleOCR 的 pyclipper unclip，用 minAreaRect 的
// 短边比例放大代替，够用即可，因为 OCR 文本框基本是矩形）。
static void UnclipRect(cv::RotatedRect &rect, float unclip_ratio) {
  float short_side = std::min(rect.size.width, rect.size.height);
  float expand = short_side * unclip_ratio * 0.5f;
  if (rect.size.width >= rect.size.height) {
    rect.size.width += expand * 2.0f;
    rect.size.height += expand;
  } else {
    rect.size.height += expand * 2.0f;
    rect.size.width += expand;
  }
}

// DB 后处理：概率图 -> 二值化 -> 连通域 -> 最小外接矩形 -> 文本框
static int PostProcess(
    std::vector<std::shared_ptr<hobot::dnn_node::DNNTensor>> &output_tensors,
    hobot::dnn_node::output_parser::Perception &perception) {
  perception.type = Perception::DET;
  if (output_tensors.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection_parser"),
                 "empty output tensors");
    return -1;
  }

  auto &tensor = output_tensors[0];
  tensor->CACHE_INVALIDATE();

  // det 输出是 [1,1,H,W]（NCHW），H=dim[2], W=dim[3]。
  // 注意不能用 get_tensor_hw（它假设 NHWC，会解析错 NCHW 输出）。
  int height = tensor->properties.validShape.dimensionSize[2];
  int width = tensor->properties.validShape.dimensionSize[3];
  RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection_parser"),
              "tensor h=%d w=%d, tensorType=%d, validShape=%dx%dx%dx%d",
              height, width, tensor->properties.tensorType,
              tensor->properties.validShape.dimensionSize[0],
              tensor->properties.validShape.dimensionSize[1],
              tensor->properties.validShape.dimensionSize[2],
              tensor->properties.validShape.dimensionSize[3]);

  // det 输出是 [1,1,H,W] float32 概率图
  float *prob = tensor->GetTensorData<float>();
  if (!prob) {
    RCLCPP_ERROR(rclcpp::get_logger("mono_ocr_detection_parser"),
                 "GetTensorData<float> failed");
    return -1;
  }

  // 统计概率图
  float max_v = 0, sum_v = 0;
  int over_thresh = 0;
  for (int i = 0; i < height * width; i++) {
    if (prob[i] > max_v) max_v = prob[i];
    sum_v += prob[i];
    if (prob[i] > g_config.thresh) over_thresh++;
  }
  RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection_parser"),
              "prob map max=%f mean=%f over_thresh=%d total=%d",
              max_v, sum_v / (height * width), over_thresh, height * width);

  // 1. 二值化
  // 1. 后处理分辨率：4K 档概率图 2176x3840 全分辨率处理实测 ~32ms；
  //    半分辨率（1/4 像素量）+ 可分离膨胀后 ~10ms，框坐标 ×step 映射回模型
  //    空间，±2px 精度损失对 rec 裁剪无感。小尺寸档保持原样。
  int step = (width >= 2000) ? 2 : 1;
  const int pw = width / step, ph = height / step;
  cv::Mat prob_mat(height, width, CV_32FC1, prob);
  cv::Mat prob_s;
  if (step > 1) {
    cv::resize(prob_mat, prob_s, cv::Size(pw, ph), 0, 0, cv::INTER_AREA);
  } else {
    prob_s = prob_mat;
  }

  cv::Mat bitmap(ph, pw, CV_8UC1);
  {
    // prob_s 是 CV_32FC1，必须按 float 读取（data 是 uchar 指针）
    const float *ps = prob_s.ptr<float>();
    for (int i = 0; i < pw * ph; i++) {
      bitmap.data[i] = (ps[i] > g_config.thresh) ? 255 : 0;
    }
  }

  // 1.5 高分辨率输入（4K 源）下字符间隙导致概率图碎裂，横向膨胀把同行
  // 字符粘连成行。仅在概率图很宽（4K 档）时启用，避免影响 letterbox/640 档。
  // 膨胀核按 step 缩小，且拆成两次一维膨胀（可分离），计算量 90->33 次/像素。
  {
    // 小尺寸概率图（640 档）文字更碎，核加倍才能桥接词间断裂
    int kx = std::max(1, width / ((width >= 1920) ? 128 : 64));
    int ky = std::max(1, width / 1280);
    cv::Mat tmp;
    cv::dilate(bitmap, tmp, cv::getStructuringElement(cv::MORPH_RECT, {kx, 1}));
    cv::dilate(tmp, bitmap, cv::getStructuringElement(cv::MORPH_RECT, {1, ky}));
  }

  // 2. 找连通域
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bitmap, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection_parser"),
              "findContours found %zu contours, thresh=%f box_thresh=%f min_size=%f",
              contours.size(), g_config.thresh, g_config.box_thresh,
              g_config.min_size);

  int num = std::min(static_cast<int>(contours.size()),
                     g_config.max_candidates);
  std::vector<Detection> dets;
  int filtered_size = 0, filtered_score = 0;
  for (int i = 0; i < num; i++) {
    // 3. 最小外接矩形（半分辨率下最小边长按 step 缩放）
    cv::RotatedRect rect = cv::minAreaRect(contours[i]);
    float short_side = std::min(rect.size.width, rect.size.height);
    if (short_side * step < g_config.min_size) {
      filtered_size++;
      continue;
    }

    // 4. unclip 膨胀
    UnclipRect(rect, g_config.unclip_ratio);

    // 5. 框内平均置信度（用降采样概率图，不是二值图 bitmap）
    cv::Rect bbox = rect.boundingRect();
    bbox &= cv::Rect(0, 0, pw, ph);
    if (bbox.width <= 0 || bbox.height <= 0) continue;
    float score = cv::mean(prob_s(bbox))[0];
    RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection_parser"),
                "contour %d: bbox=%dx%d score=%f short_side=%f",
                i, bbox.width, bbox.height, score, short_side);
    if (score < g_config.box_thresh) {
      filtered_score++;
      continue;
    }

    // 6. 输出框（半分辨率坐标 ×step 映射回模型坐标系，node 再映射回原图）
    cv::Point2f pts[4];
    rect.points(pts);
    float xmin = std::min({pts[0].x, pts[1].x, pts[2].x, pts[3].x}) * step;
    float ymin = std::min({pts[0].y, pts[1].y, pts[2].y, pts[3].y}) * step;
    float xmax = std::max({pts[0].x, pts[1].x, pts[2].x, pts[3].x}) * step;
    float ymax = std::max({pts[0].y, pts[1].y, pts[2].y, pts[3].y}) * step;

    const char *name = "text";
    if (!g_config.class_names.empty() && !g_config.class_names[0].empty())
      name = g_config.class_names[0].c_str();

    dets.emplace_back(0, score,
                      hobot::dnn_node::output_parser::Bbox(
                          xmin, ymin, xmax, ymax),
                      name);
  }
  RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection_parser"),
              "after filter: total=%d size_filtered=%d score_filtered=%d dets=%zu",
              num, filtered_size, filtered_score, dets.size());

  // 7. NMS 去重叠框：低阈值召回提升后，高密度小字区会出现 IoU 很高的
  // 重复框；用 NMS 按 score 去重，而不是靠抬高阈值牺牲召回
  if (g_config.nms_iou > 0 && dets.size() > 1) {
    std::vector<Detection> nms_result;
    nms(dets, g_config.nms_iou, static_cast<int>(dets.size()), nms_result);
    RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection_parser"),
                "nms: %zu -> %zu (iou=%f)",
                dets.size(), nms_result.size(), g_config.nms_iou);
    dets.swap(nms_result);
  }

  perception.det = dets;
  return 0;
}

int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    std::shared_ptr<DnnParserResult> &result) {
  if (!result) result = std::make_shared<DnnParserResult>();
  // det 模型只有 1 个输出，output_order 为空时用原顺序
  if (!g_config.output_order.empty()) {
    SortByOrder(node_output->output_tensors, g_config.output_order);
  }
  int ret = PostProcess(node_output->output_tensors, result->perception);
  if (ret != 0)
    RCLCPP_INFO(rclcpp::get_logger("mono_ocr_detection_parser"),
                "postprocess error, code=%d", ret);
  return ret;
}

}  // namespace mono_ocr_detection
