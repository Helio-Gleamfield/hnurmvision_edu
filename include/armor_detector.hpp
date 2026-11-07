#pragma once
#include <vector>

#include "types.hpp"
namespace training {
std::vector<Armor> detect_opencv(const cv::Mat& image);
cv::Mat draw_armors(const cv::Mat& image, const std::vector<Armor>& armors);
}  // namespace training
