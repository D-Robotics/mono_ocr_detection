#ifndef MONO_OCR_DETECTION_NMS_H_
#define MONO_OCR_DETECTION_NMS_H_

#include <vector>

#include "dnn_node/util/output_parser/perception_common.h"

using hobot::dnn_node::output_parser::Detection;

namespace mono_ocr_detection {

void nms(std::vector<Detection> &input, float iou_threshold,
         int top_k, std::vector<Detection> &result, bool suppress = false);

}  // namespace mono_ocr_detection

#endif  // MONO_OCR_DETECTION_NMS_H_
