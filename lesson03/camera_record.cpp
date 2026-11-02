#include <cstdlib>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "armor_detector.hpp"
#include "io_context.hpp"
#include "lessons.hpp"
int main(int argc, char** argv) try {
  if (argc < 3)
    throw std::runtime_error("camera_record CONFIG OUTPUT_AVI [SECONDS]");
  auto config = training::load_config(argv[1]);
  training::io::IoContext devices{config};
  devices.open();
  if (config.display) {
    cv::namedWindow("Lesson03 live camera", cv::WINDOW_NORMAL);
    cv::resizeWindow("Lesson03 live camera", 960, 720);
  }
  cv::VideoWriter output, raw;
  const double recording_fps = std::getenv("RM_RECORD_FPS")
                                   ? std::stod(std::getenv("RM_RECORD_FPS"))
                                   : 30.0;
  int count = 0, hits = 0;
  auto start = training::Clock::now();
  double seconds = argc > 3 ? std::stod(argv[3]) : 20;
  while (std::chrono::duration<double>(training::Clock::now() - start).count() <
         seconds) {
    auto input = devices.read();
    if (input) {
      auto armors = training::lesson03_detection(input->image);
      auto canvas = training::draw_armors(input->image, armors);
      if (!output.isOpened()) {
        output.open(argv[2], cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                    recording_fps, canvas.size());
        raw.open(std::string(argv[2]) + "-raw.avi",
                 cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), recording_fps,
                 canvas.size());
        if (!output.isOpened() || !raw.isOpened())
          throw std::runtime_error("writer failed");
      }
      output.write(canvas);
      raw.write(input->image);
      ++count;
      hits += !armors.empty();
      if (count == 60 || count == 180)
        cv::imwrite(std::string(argv[2]) + "-" + std::to_string(count) + ".jpg",
                    canvas);
      if (config.display) cv::imshow("Lesson03 live camera", canvas);
    }
    if (config.display && cv::waitKey(1) == 27) break;
  }
  devices.close();
  std::cout << "{\"frames\":" << count << ",\"frames_with_detection\":" << hits
            << "}\n";
  return count ? 0 : 1;
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
