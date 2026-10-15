#ifndef MONO_OCR_DETECTION_OUTPUT_PARSER_H_
#define MONO_OCR_DETECTION_OUTPUT_PARSER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rapidjson/document.h"

#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"

using hobot::dnn_node::output_parser::Bbox;
using hobot::dnn_node::output_parser::Detection;
using hobot::dnn_node::output_parser::DnnParserResult;
using hobot::dnn_node::output_parser::Perception;

namespace mono_ocr_detection {

// Parse the JSON workconfig to initialize model/postprocess parameters.
// Reads: model_output_count, class_num, reg_max, strides, score_threshold,
// nms_threshold, nms_top_k, is_performance, output_order.
// Class names are injected separately via SetClassNames().
int LoadConfig(const rapidjson::Document &document);

// Override the parser's class-name table (e.g. loaded from a vocabulary json).
// Empty input is ignored so the parser's default table remains intact.
void SetClassNames(const std::vector<std::string> &names);

// Run post-processing on the model output tensors and fill `output`.
int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    std::shared_ptr<DnnParserResult> &output);

}  // namespace mono_ocr_detection
#endif  // MONO_OCR_DETECTION_OUTPUT_PARSER_H_
