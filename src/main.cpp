#include "armor_detector.hpp"
#include "aiming.hpp"
#include "evaluation.hpp"
#include "io_context.hpp"
#include "lessons.hpp"
#include <csignal>
#include <iostream>
#include <opencv2/highgui.hpp>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }
} // namespace

int main(int argc, char **argv) try {
  std::cout << std::unitbuf;
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  const auto config =
      training::load_config(argc > 1 ? argv[1] : "configs/simulation.yaml");
  const auto calibration = training::load_calibration(config);
  training::io::IoContext devices{config};
  training::TrackingState tracking;
  training::FireControl fire;
  tracking.parameters.use_tilt_prior = true;
  devices.open();
  auto last_input=training::Clock::now();
  training::Lesson04Evaluation evaluation;
  if (devices.config.display) {
    cv::namedWindow("RoboMaster camera (view only; control in simulator)",
                    cv::WINDOW_NORMAL);
    cv::resizeWindow("RoboMaster camera (view only; control in simulator)", 640,
                     480);
  }

  while (!interrupted) {
    evaluation.poll();
    const auto input = devices.read();
    training::Command command{};
    cv::Mat annotated;
    if (input) {
      last_input=training::Clock::now();
      const auto armors = training::lesson03_detection(input->image);
      annotated = training::draw_armors(input->image, armors);
      const auto target = training::lesson04_tracking(
          armors, input->timestamp, input->imu_q, calibration, tracking);
      evaluation.observe(*input, tracking, calibration);
      annotated = training::draw_tracking(annotated, *input, tracking,
                                          calibration, evaluation.view());
      if (target) {
        const auto solution=training::solve_aim(*target,input->timestamp,input->bullet_speed,input->imu_q);
        command=fire.update(solution,input->imu_q,
            tracking.phase==training::TrackingState::Phase::Tracking && bool(tracking.observation),tracking.generation);
      } else fire.reset();
    }
    if (interrupted)
      break;
    if (input) devices.write(command);
    else if (training::Clock::now()-last_input>std::chrono::milliseconds(200)) {
      fire.reset();devices.write(training::Command{});
    }

    if (devices.config.display) {
      if (input)
        cv::imshow("RoboMaster camera (view only; control in simulator)",
                   annotated);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q')
        break;
    }
  }

  devices.close();
  if (devices.config.display)
    cv::destroyAllWindows();
  return 0;
} catch (const std::exception &error) {
  std::cerr << "training: " << error.what() << '\n';
  return 1;
}
