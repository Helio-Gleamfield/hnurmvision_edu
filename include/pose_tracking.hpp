#pragma once
#include "tracking_state.hpp"
namespace training {
std::vector<cv::Point3d>
armor_object_points(bool large, const TrackingParameters &parameters);
std::optional<PoseObservation> solve_armor_pose(const Armor &,
                                                const CameraCalibration &,
                                                const Eigen::Quaterniond &,
                                                const TrackingParameters &);
std::optional<std::array<cv::Point2f, 4>>
predicted_corners(const TrackingState &, const CameraCalibration &,
                  const Eigen::Quaterniond &);
const char *tracking_phase_name(TrackingState::Phase);
} // namespace training
