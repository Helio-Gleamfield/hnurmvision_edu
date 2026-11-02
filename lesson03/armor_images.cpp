#include <algorithm>
#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

#include "armor_detector.hpp"
int main(int argc, char** argv) try {
  if (argc != 2) throw std::runtime_error("armor_images IMAGE_DIRECTORY");
  std::vector<std::filesystem::path> files;
  for (auto& e : std::filesystem::directory_iterator(argv[1]))
    if (e.path().extension() == ".jpg" || e.path().extension() == ".png")
      files.push_back(e.path());
  std::sort(files.begin(), files.end());
  for (const auto& path : files) {
    auto image = cv::imread(path.string());
    if (image.empty()) throw std::runtime_error("Unreadable image");
    auto armors = training::detect_opencv(image);
    std::cout << "{\"image\":\"" << path.filename().string()
              << "\",\"boxes\":[";
    bool first = true;
    for (auto& a : armors) {
      if (!first) std::cout << ',';
      first = false;
      float x1 = image.cols, y1 = image.rows, x2 = 0, y2 = 0;
      for (auto p : a.corners) {
        x1 = std::min(x1, p.x);
        y1 = std::min(y1, p.y);
        x2 = std::max(x2, p.x);
        y2 = std::max(y2, p.y);
      }
      std::cout << '[' << x1 << ',' << y1 << ',' << x2 << ',' << y2 << ']';
    }
    std::cout << "]}\n";
  }
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
