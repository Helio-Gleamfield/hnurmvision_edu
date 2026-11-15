#pragma once
#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

#include "types.hpp"
namespace training {
struct TrackingParameters {
  double small_width_m = .135, large_width_m = .225, height_m = .055;
  double prediction_ms = 250, measurement_sigma_m = .025,
         acceleration_sigma_mps2 = 4;
  double max_reprojection_px = 3, match_distance_m = .45, reset_gap_s = .3,
         lost_timeout_s = .18;
  int confirm_frames = 3;
  bool use_tilt_prior = false;
  double armor_tilt_deg = 15;
};
struct PoseObservation {
  Armor armor;
  cv::Vec3d rvec{}, tvec{};
  Eigen::Matrix3d rotation_camera = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double reprojection_px = 0, image_area = 0;
};
struct TrackingState {
  TrackingParameters parameters;
  enum class Phase { Lost, Detecting, Tracking, TempLost };
  Phase phase = Phase::Lost;
  bool initialized = false;
  int hits = 0;
  unsigned generation = 0;
  TimePoint timestamp{}, last_observed{};
  Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Identity();
  std::optional<PoseObservation> observation;
  std::string diagnostic = "no observation";
};
} // namespace training
