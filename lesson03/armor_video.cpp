#include <chrono>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "armor_detector.hpp"
#include "lessons.hpp"
int main(int argc, char** argv) try {
  if (argc < 3) {
    std::cerr
        << "Usage: armor_video INPUT_VIDEO OUTPUT_AVI [MAX_FRAMES] [show]\n";
    return 2;
  }
  cv::VideoCapture input(argv[1]);
  if (!input.isOpened()) throw std::runtime_error("Cannot open input");
  double fps = input.get(cv::CAP_PROP_FPS);
  if (!(fps > 0 && fps < 240)) fps = 30;
  cv::Size size(int(input.get(cv::CAP_PROP_FRAME_WIDTH)),
                int(input.get(cv::CAP_PROP_FRAME_HEIGHT)));
  cv::VideoWriter output(argv[2], cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                         fps, size);
  if (!output.isOpened()) throw std::runtime_error("Cannot open writer");
  int count = 0, detected = 0, instances = 0,
      limit = argc > 3 ? std::stoi(argv[3]) : 0;
  cv::Mat image;
  auto start = std::chrono::steady_clock::now();
  while ((!limit || count < limit) && input.read(image)) {
    auto armors = training::lesson03_detection(image);
    auto canvas = training::draw_armors(image, armors);
    ++count;
    detected += !armors.empty();
    instances += armors.size();
    cv::putText(canvas,
                "frame " + std::to_string(count) + " detections " +
                    std::to_string(armors.size()),
                {12, 25}, cv::FONT_HERSHEY_SIMPLEX, .6, {0, 255, 255}, 2);
    output.write(canvas);
    if (count == 1 || count == 60 || count == 180)
      cv::imwrite(std::string(argv[2]) + "-" + std::to_string(count) + ".jpg",
                  canvas);
    if (argc > 4) {
      cv::imshow("Lesson03 camera detections", canvas);
      if (cv::waitKey(1) == 27) break;
    }
  }
  output.release();
  std::cout << "{\"frames\":" << count
            << ",\"frames_with_detection\":" << detected
            << ",\"instances\":" << instances << ",\"seconds\":"
            << std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             start)
                   .count()
            << "}\n";
  return count ? 0 : 1;
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
