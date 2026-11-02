#include "lessons.hpp"
#include "armor_detector.hpp"
#include "io_context.hpp"
#include <opencv2/highgui.hpp>
#include <csignal>
#include <iostream>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }
}

int main(int argc, char** argv) try {
  std::cout << std::unitbuf;
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  const auto config = training::load_config(argc > 1 ? argv[1] : "configs/simulation.yaml");
  const auto calibration = training::load_calibration(config);
  training::io::IoContext devices{config};
  training::TrackingState tracking;
  devices.open();
  if (devices.config.display) {
    cv::namedWindow("RoboMaster camera (view only; control in simulator)", cv::WINDOW_NORMAL);
    cv::resizeWindow("RoboMaster camera (view only; control in simulator)", 640, 480);
  }

  while (!interrupted) {
    const auto input = devices.read();
    training::Command command{};
    cv::Mat annotated;
    if (input) {
      const auto armors = training::lesson03_detection(input->image);
      annotated=training::draw_armors(input->image,armors);
      const auto target = training::lesson04_tracking(
          armors, input->timestamp, input->imu_q, calibration, tracking);
      if (target)
        command = training::lesson05_aiming(*target, input->timestamp, input->bullet_speed);
    }
    if (interrupted) break;
    devices.write(command);

    if (devices.config.display) {
      if (input) cv::imshow("RoboMaster camera (view only; control in simulator)", annotated);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q') break;
    }
  }

  devices.close();
  if (devices.config.display) cv::destroyAllWindows();
  return 0;
} catch (const std::exception& error) {
  std::cerr << "training: " << error.what() << '\n';
  return 1;
}
