#pragma once
#include "types.hpp"
#include <optional>
#include <string>
namespace training {
struct BallisticSolution { double elevation = 0, flight_s = 0; };
std::optional<BallisticSolution> solve_ballistic(double speed, double horizontal,
                                               double height, double gravity = 9.81);
struct AimParameters {
  double delay_s = 0, gravity = 9.81, mount_elevation = 25 * kPi / 180;
  double max_age_s = .2, pitch_limit = .785, convergence_s = .0001;
  int max_iterations = 16;
  // Rigid model extrinsic in unmounted gimbal FLU coordinates.
  Eigen::Vector3d muzzle_gimbal{.01070404, .00183100, .11028814};
};
struct AimSolution {
  Command command;
  Eigen::Vector3d intercept = Eigen::Vector3d::Zero();
  double observation_age_s = 0, flight_s = 0, lead_s = 0, elevation = 0;
  int iterations = 0;
  std::string reason = "no solution";
};
AimSolution solve_aim(const Target &, TimePoint timestamp, double bullet_speed,
                      const Eigen::Quaterniond &imu, const AimParameters & = {},
                      TimePoint now = Clock::now());
class FireControl {
public:
  Command update(const AimSolution &, const Eigen::Quaterniond &feedback,
                 bool trusted, unsigned generation);
  void reset();
  double yaw_error = 0, pitch_error = 0;
  std::string reason = "waiting for target";
private:
  std::optional<Command> previous_;
  unsigned generation_ = 0;
};
} // namespace training
