#include <iostream>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "armor_detector.hpp"
#include "lessons.hpp"
void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
int main() {
  check(training::detect_opencv({}).empty(), "empty");
  cv::Mat black = cv::Mat::zeros(240, 400, CV_8UC3);
  check(training::detect_opencv(black).empty(), "black");
  for (auto color : {cv::Scalar(255, 40, 10), cv::Scalar(10, 40, 255)}) {
    auto image = black.clone();
    cv::rectangle(image, {100, 80, 8, 60}, color, -1);
    cv::rectangle(image, {230, 80, 8, 60}, color, -1);
    auto original = image.clone();
    auto found = training::detect_opencv(image);
    check(found.size() == 1, "pair count");
    auto a = found[0];
    check(a.class_id == -1 && a.confidence > 0 && a.confidence <= 1, "quality");
    check(a.corners[0].x < a.corners[1].x && a.corners[0].y < a.corners[3].y,
          "corner ordering");
    check(cv::norm(original, image) == 0, "input immutable");
    check(cv::norm(image, training::draw_armors(image, found)) > 0,
          "overlay drawn");
  }
  auto image = black.clone();
  cv::rectangle(image, {100, 80, 8, 60}, {255, 0, 0}, -1);
  cv::rectangle(image, {230, 80, 8, 60}, {0, 0, 255}, -1);
  check(training::detect_opencv(image).empty(), "mixed colors rejected");
  bool threw = false;
  try {
    training::detect_opencv(cv::Mat::zeros(40, 40, CV_8UC1));
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  check(threw, "bad input rejected");
  std::cout << "PASS: empty, black, red/blue pair, ordering, score, immutable "
               "input, overlay, mixed colors, type validation\n";
}
