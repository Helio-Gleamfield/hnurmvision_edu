#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "armor_detector.hpp"
#include "evaluation.hpp"
#include "io_context.hpp"
#include "lessons.hpp"
namespace {
volatile std::sig_atomic_t stopped = 0;
void signal_stop(int) { stopped = 1; }
} // namespace
int main(int argc, char **argv) try {
  std::string out = "", config_path = "configs/simulation.yaml";
  double seconds = 25, horizon = 250;
  bool show = true;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto value = [&]() {
      if (++i >= argc)
        throw std::runtime_error("missing option value");
      return std::string(argv[i]);
    };
    if (a == "--output")
      out = value();
    else if (a == "--seconds")
      seconds = std::stod(value());
    else if (a == "--horizon-ms")
      horizon = std::stod(value());
    else if (a == "--config")
      config_path = value();
    else if (a == "--headless")
      show = false;
    else
      throw std::runtime_error("Unknown argument: " + a);
  }
  if (!std::isfinite(seconds) || !std::isfinite(horizon) || seconds <= 0 ||
      seconds > 600 || horizon < 0 || horizon > 2000)
    throw std::runtime_error("Invalid seconds/horizon");
  if (!out.empty()) {
    if (std::filesystem::exists(out))
      throw std::runtime_error("Output exists; choose a fresh name");
    std::filesystem::create_directories(out + "/frames");
  }
  std::signal(SIGINT, signal_stop);
  std::signal(SIGTERM, signal_stop);
  auto config = training::load_config(config_path);
  auto calibration = training::load_calibration(config);
  training::io::IoContext devices{config};
  devices.open();
  training::TrackingState tracking;
  tracking.parameters.prediction_ms = horizon;
  tracking.parameters.use_tilt_prior = true;
  training::Lesson04Evaluation eval(out.empty() ? "" : out + "/evaluation");
  if (show) {
    cv::namedWindow("Lesson04 Pose + Tracking", cv::WINDOW_NORMAL);
    cv::resizeWindow("Lesson04 Pose + Tracking", 1100, 825);
  }
  std::ofstream frames;
  if (!out.empty())
    frames.open(out + "/frames.csv");
  if (frames)
    frames << "filename,acquisition_ros_ns\n";
  std::optional<training::TimePoint> first;
  std::int64_t last_record = 0;
  std::size_t frame_count = 0, record_count = 0, tracks = 0;
  auto start = training::Clock::now();
  auto last_input_arrival = start;
  while (!stopped) {
    eval.poll();
    auto input = devices.read();
    if (input) {
      last_input_arrival = training::Clock::now();
      if (!first)
        first = input->timestamp;
      auto armors = training::lesson03_detection(input->image);
      auto target = training::lesson04_tracking(
          armors, input->timestamp, input->imu_q, calibration, tracking);
      eval.observe(*input, tracking, calibration);
      ++frame_count;
      tracks += bool(target);
      auto canvas = training::draw_armors(input->image, armors);
      canvas = training::draw_tracking(canvas, *input, tracking, calibration,
                                       eval.view());
      if (frames && input->acquisition_ros_ns - last_record >= 50000000LL) {
        std::string name = cv::format("%06zu.jpg", record_count++);
        cv::imwrite(out + "/frames/" + name, canvas,
                    {cv::IMWRITE_JPEG_QUALITY, 90});
        frames << name << ',' << input->acquisition_ros_ns << '\n';
        last_record = input->acquisition_ros_ns;
      }
      if (show)
        cv::imshow("Lesson04 Pose + Tracking", canvas);
      if (std::chrono::duration<double>(input->timestamp - *first).count() >=
          seconds)
        break;
    }
    if (first && std::chrono::duration<double>(training::Clock::now() -
                                               last_input_arrival)
                         .count() > 5)
      throw std::runtime_error("Camera stream stopped for 5 seconds");
    if (!first &&
        std::chrono::duration<double>(training::Clock::now() - start).count() >
            25)
      throw std::runtime_error(
          "No synchronized camera input after 25 seconds; start simulator in "
          "matching domain");
    if (show) {
      int key = cv::waitKey(1);
      if (key == 27 || key == 'q')
        break;
      if (key == ']')
        tracking.parameters.prediction_ms =
            std::min(2000., tracking.parameters.prediction_ms + 50);
      if (key == '[')
        tracking.parameters.prediction_ms =
            std::max(0., tracking.parameters.prediction_ms - 50);
    }
  }
  devices.close();
  eval.poll();
  if (!out.empty())
    eval.write_summary(out + "/summary.yaml");
  auto e = eval.view();
  std::cout << "frames=" << frame_count << " tracked=" << tracks
            << " pose_truth_pairs=" << e.pose_samples
            << " delayed_checks=" << e.prediction_samples << "\n";
  if (show)
    cv::destroyAllWindows();
  return frame_count > 0 && e.pose_samples > 0 && e.prediction_samples > 0 ? 0
                                                                           : 2;
} catch (const std::exception &e) {
  std::cerr << "lesson04: " << e.what() << '\n';
  return 1;
}
