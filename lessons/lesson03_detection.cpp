#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>

#include "armor_detector.hpp"
#include "lessons.hpp"

namespace training {
namespace {
struct Light {
  cv::Point2f top, bottom, center;
  float length, width;
  int color;
};
}  // namespace
std::vector<Armor> detect_opencv(const cv::Mat& image) {
  if (image.empty()) return {};
  if (image.type() != CV_8UC3)
    throw std::invalid_argument("Expected CV_8UC3 BGR image");
  std::vector<Light> lights;
  std::vector<cv::Mat> channels;
  cv::split(image, channels);
  for (int color = 0; color < 2; ++color) {
    cv::Mat difference, mask, bright;
    cv::subtract(channels[color == 0 ? 0 : 2], channels[color == 0 ? 2 : 0],
                 difference);
    cv::threshold(difference, mask, 45, 255, cv::THRESH_BINARY);
    cv::threshold(channels[color == 0 ? 0 : 2], bright, 110, 255,
                  cv::THRESH_BINARY);
    cv::bitwise_and(mask, bright, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
      if (cv::contourArea(contour) < 5 || contour.size() < 4) continue;
      auto rect = cv::minAreaRect(contour);
      cv::Point2f p[4];
      rect.points(p);
      float len = std::max(rect.size.width, rect.size.height),
            width = std::min(rect.size.width, rect.size.height);
      if (width < 1 || len < 5 || len / width < 1.7 || len / width > 20)
        continue;
      cv::Point2f v = p[1] - p[0];
      if (cv::norm(v) < len * .9) v = p[2] - p[1];
      if (v.y < 0) v = -v;
      if (std::abs(v.x) > std::abs(v.y) * .85) continue;
      v *= len / (2 * cv::norm(v));
      lights.push_back(
          {rect.center - v, rect.center + v, rect.center, len, width, color});
    }
  }
  struct Candidate {
    Armor armor;
    size_t left, right;
  };
  std::vector<Candidate> candidates;
  for (size_t i = 0; i < lights.size(); ++i)
    for (size_t j = i + 1; j < lights.size(); ++j) {
      auto l = lights[i], r = lights[j];
      if (l.center.x > r.center.x) std::swap(l, r);
      if (l.color != r.color) continue;
      float mean = (l.length + r.length) / 2, dx = r.center.x - l.center.x,
            dy = std::abs(r.center.y - l.center.y);
      float ratio = dx / mean, length_ratio = std::min(l.length, r.length) /
                                              std::max(l.length, r.length);
      auto vl = l.bottom - l.top, vr = r.bottom - r.top;
      float angle =
          std::acos(std::clamp(vl.dot(vr) / (l.length * r.length), -1.f, 1.f));
      if (length_ratio < .55 || dy / mean > .65 || ratio < .8 || ratio > 5.3 ||
          angle > .4)
        continue;
      // Reject a pair spanning a third same-color light.
      bool middle = false;
      for (size_t k = 0; k < lights.size(); ++k)
        if (k != i && k != j && lights[k].color == l.color &&
            lights[k].center.x > l.center.x &&
            lights[k].center.x < r.center.x &&
            std::abs(lights[k].center.y - (l.center.y + r.center.y) / 2) <
                mean * .5)
          middle = true;
      if (middle) continue;
      Armor a;
      a.corners = {l.top, r.top, r.bottom, l.bottom};
      a.large_armor = ratio > 3.2;
      a.class_id = -1;
      // Geometric quality score, not a calibrated neural probability.
      a.confidence = std::clamp(
          double(length_ratio * std::exp(-dy / mean - angle)), 0., 1.);
      candidates.push_back({a, i, j});
    }
  std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) {
              return a.armor.confidence > b.armor.confidence;
            });
  std::vector<bool> used(lights.size());
  std::vector<Armor> result;
  for (const auto& c : candidates)
    if (!used[c.left] && !used[c.right]) {
      used[c.left] = used[c.right] = true;
      result.push_back(c.armor);
    }
  return result;
}
std::vector<Armor> lesson03_detection(const cv::Mat& image) {
  const char* backend = std::getenv("RM_DETECTOR");
  if (!backend || std::string(backend) == "opencv") return detect_opencv(image);
  if (std::string(backend) == "neural") return detect_neural(image);
  throw std::invalid_argument("RM_DETECTOR must be opencv or neural");
}
cv::Mat draw_armors(const cv::Mat& image, const std::vector<Armor>& armors) {
  cv::Mat canvas = image.clone();
  for (const auto& a : armors) {
    std::vector<cv::Point> poly;
    for (auto p : a.corners) poly.emplace_back(cvRound(p.x), cvRound(p.y));
    cv::polylines(canvas, poly, true, {0, 255, 0}, 2, cv::LINE_AA);
    for (int i = 0; i < 4; ++i) {
      cv::circle(canvas, a.corners[i], 3, {0, 255, 255}, -1);
      cv::putText(canvas, std::to_string(i), a.corners[i],
                  cv::FONT_HERSHEY_SIMPLEX, .4, {255, 255, 255}, 1);
    }
    const std::string label =
        (a.class_id < 0 ? "geometry" : "class=" + std::to_string(a.class_id)) +
        cv::format(" %.2f", a.confidence) +
        (a.large_armor ? " large" : " small");
    cv::putText(canvas, label, poly[0] - cv::Point(0, 8),
                cv::FONT_HERSHEY_SIMPLEX, .45, {0, 255, 0}, 1, cv::LINE_AA);
  }
  return canvas;
}
}  // namespace training
