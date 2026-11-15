#pragma once
#include <memory>

#include "pose_tracking.hpp"
namespace training {
struct EvaluationView {
  bool truth_available = false;
  Eigen::Vector3d true_camera = Eigen::Vector3d::Zero(),
                  true_stable = Eigen::Vector3d::Zero();
  double pose_error_m = 0, rotation_error_deg = 0, stable_error_m = 0;
  bool prediction_available = false;
  double prediction_error_m = 0, prediction_horizon_ms = 0,
         evaluation_lag_ms = 0;
  std::uint64_t pose_samples = 0, prediction_samples = 0, unmatched_frames = 0;
};
class Lesson04Evaluation {
public:
  explicit Lesson04Evaluation(const std::string &csv_prefix = "");
  ~Lesson04Evaluation();
  void poll();
  void observe(const SensorData &, const TrackingState &,
               const CameraCalibration &);
  EvaluationView view() const;
  void write_summary(const std::string &path) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
cv::Mat draw_tracking(const cv::Mat &, const SensorData &,
                      const TrackingState &, const CameraCalibration &,
                      const EvaluationView &);
} // namespace training
