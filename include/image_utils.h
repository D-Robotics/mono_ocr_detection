#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

#include <memory>
#include <string>
#include <vector>

#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node_data.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

using hobot::dnn_node::DNNTensor;

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

static std::vector<cv::Scalar> colors{
    cv::Scalar(255, 0, 0),    // red
    cv::Scalar(255, 255, 0),  // yellow
    cv::Scalar(0, 255, 0),    // green
    cv::Scalar(0, 0, 255),    // blue
};

class ImageUtils {
 public:
  static int Render(
    const std::shared_ptr<hobot::dnn_node::NV12PyramidInput> &pyramid,
    const ai_msgs::msg::PerceptionTargets::UniquePtr &ai_msg,
    const int img_h,
    const int img_w,
    const std::string folder);
};

#endif  // IMAGE_UTILS_H
