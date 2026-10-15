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

#include <algorithm>
#include <vector>

#include "dnn_node/util/output_parser/perception_common.h"

using hobot::dnn_node::output_parser::Detection;

namespace mono_ocr_detection {

void nms(std::vector<Detection> &input, float iou_threshold,
         int top_k, std::vector<Detection> &result, bool suppress) {
  std::sort(input.begin(), input.end(),
            [](const Detection &a, const Detection &b) { return a.score > b.score; });

  std::vector<bool> skip(input.size(), false);
  std::vector<float> areas(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    float w = input[i].bbox.xmax - input[i].bbox.xmin;
    float h = input[i].bbox.ymax - input[i].bbox.ymin;
    areas[i] = w * h;
  }

  int count = 0;
  for (size_t i = 0; count < top_k && i < skip.size(); i++) {
    if (skip[i]) continue;
    skip[i] = true;
    ++count;
    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) continue;
      if (!suppress && input[i].id != input[j].id) continue;
      float xx1 = std::max(input[i].bbox.xmin, input[j].bbox.xmin);
      float yy1 = std::max(input[i].bbox.ymin, input[j].bbox.ymin);
      float xx2 = std::min(input[i].bbox.xmax, input[j].bbox.xmax);
      float yy2 = std::min(input[i].bbox.ymax, input[j].bbox.ymax);
      if (xx2 > xx1 && yy2 > yy1) {
        float inter = (xx2 - xx1) * (yy2 - yy1);
        float iou = inter / (areas[j] + areas[i] - inter);
        if (iou > iou_threshold) skip[j] = true;
      }
    }
    result.push_back(input[i]);
  }
}

}  // namespace mono_ocr_detection
