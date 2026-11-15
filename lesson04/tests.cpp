#include <Eigen/Eigenvalues>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <stdexcept>

#include "lessons.hpp"
#include "pose_tracking.hpp"
using namespace training;
int checks = 0;
void require(bool v, const char *s) {
  ++checks;
  if (!v)
    throw std::runtime_error(s);
}
CameraCalibration calibration() {
  CameraCalibration c;
  c.matrix = (cv::Mat_<double>(3, 3) << 1303.67532368147, 0, 720, 0,
              1303.67532368147, 540, 0, 0, 1);
  c.distortion = cv::Mat::zeros(1, 5, CV_64F);
  double a = 25 * kPi / 180;
  c.camera_to_gimbal << 0, std::sin(a), std::cos(a), -1, 0, 0, 0, -std::cos(a),
      std::sin(a);
  c.camera_to_gimbal_translation = {.02, -.01, .03};
  return c;
}
Armor from_camera(const cv::Vec3d &r, const cv::Vec3d &t,
                  const CameraCalibration &c, bool large = false) {
  Armor a;
  a.large_armor = large;
  a.confidence = .95;
  std::vector<cv::Point2d> v;
  cv::projectPoints(armor_object_points(large, {}), r, t, c.matrix,
                    c.distortion, v);
  std::copy(v.begin(), v.end(), a.corners.begin());
  return a;
}
Armor from_stable(const Eigen::Vector3d &p, const Eigen::Matrix3d &Rstable,
                  const Eigen::Quaterniond &q, const CameraCalibration &c) {
  Eigen::Matrix3d R = c.camera_to_gimbal.transpose() *
                      q.conjugate().toRotationMatrix() * Rstable;
  Eigen::Vector3d t = c.camera_to_gimbal.transpose() *
                      (q.conjugate() * p - c.camera_to_gimbal_translation);
  cv::Mat m(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      m.at<double>(i, j) = R(i, j);
  cv::Vec3d rv;
  cv::Rodrigues(m, rv);
  return from_camera(rv, {t.x(), t.y(), t.z()}, c);
}
int main() try {
  auto c = calibration();
  Eigen::Quaterniond q(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()));
  cv::Vec3d rv(.12, -.18, .03), tv(.1, .15, 3.1);
  for (bool large : {false, true}) {
    auto a = from_camera(rv, tv, c, large);
    auto p = solve_armor_pose(a, c, q, {});
    require(bool(p), "PnP solution");
    require(cv::norm(p->tvec - tv) < .0001, "PnP meter scale and translation");
    require(cv::norm(p->rvec - rv) < .001, "PnP rotation and corner order");
    Eigen::Vector3d expected =
        q * (c.camera_to_gimbal * Eigen::Vector3d(tv[0], tv[1], tv[2]) +
             c.camera_to_gimbal_translation);
    require((p->position - expected).norm() < .0001,
            "camera to gimbal translation then IMU rotation");
  }
  {
    Eigen::Quaterniond g(
        Eigen::AngleAxisd(-kPi / 2, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(25 * kPi / 180, Eigen::Vector3d::UnitY()));
    double angle = 15 * kPi / 180;
    Eigen::Matrix3d shape;
    shape.col(0) = Eigen::Vector3d(-1, 0, 0);
    shape.col(1) = Eigen::Vector3d(0, std::sin(angle), -std::cos(angle));
    shape.col(2) = shape.col(0).cross(shape.col(1));
    Eigen::Vector3d stable =
        g * (c.camera_to_gimbal * Eigen::Vector3d(.01, .2, 2.7) +
             c.camera_to_gimbal_translation);
    auto board = from_stable(stable, shape, g, c);
    TrackingParameters options;
    options.use_tilt_prior = true;
    auto estimate = solve_armor_pose(board, c, g, options);
    require(bool(estimate), "physical tilt hypothesis");
    require((estimate->position - stable).norm() < .002,
            "tilt prior translation");
  }
  auto a = from_camera(rv, tv, c);
  a.corners[1] = a.corners[0];
  require(!solve_armor_pose(a, c, q, {}), "degenerate polygon rejected");
  a = from_camera(rv, tv, c);
  a.corners[0].x = std::numeric_limits<float>::quiet_NaN();
  require(!solve_armor_pose(a, c, q, {}), "NaN rejected");
  require(!solve_armor_pose(from_camera(rv, tv, c), c,
                            Eigen::Quaterniond(0, 0, 0, 0), {}),
          "zero quaternion rejected");
  TimePoint start = Clock::now();
  TrackingState s;
  Eigen::Vector3d p0 = c.camera_to_gimbal * Eigen::Vector3d(0, .1, 3) +
                       c.camera_to_gimbal_translation,
                  v(.1, .4, -.02);
  Eigen::Matrix3d obj = c.camera_to_gimbal;
  std::optional<Target> target;
  for (int i = 0; i < 100; ++i) {
    double time = i / 30.;
    auto now = start + std::chrono::microseconds(std::llround(time * 1e6));
    Eigen::Quaterniond pose(
        Eigen::AngleAxisd(.06 * std::sin(time), Eigen::Vector3d::UnitZ()));
    auto observation = from_stable(p0 + v * time, obj, pose, c);
    target = lesson04_tracking({observation}, now, pose, c, s);
    if (i < 2)
      require(!target, "confirmation delay");
  }
  require(bool(target), "constant velocity target tracked");
  require((target->position - (p0 + v * (99 / 30.))).norm() < .01,
          "filtered current position");
  require((target->velocity - v).norm() < .03,
          "velocity in stable frame under rotating camera");
  require(s.covariance.isApprox(s.covariance.transpose(), 1e-8),
          "symmetric covariance");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(s.covariance);
  require(eig.eigenvalues().minCoeff() > -1e-10, "PSD Joseph covariance");
  auto previous = s.x;
  require(
      !lesson04_tracking({}, s.timestamp, Eigen::Quaterniond::Identity(), c, s),
      "duplicate timestamp rejected");
  require(s.x.isApprox(previous),
          "duplicate timestamp does not advance filter");
  auto t = s.timestamp + std::chrono::milliseconds(34);
  require(!lesson04_tracking({}, t, Eigen::Quaterniond::Identity(), c, s),
          "no observation gives no actionable target");
  require(s.phase == TrackingState::Phase::TempLost, "temporary loss state");
  t += std::chrono::milliseconds(220);
  lesson04_tracking({}, t, Eigen::Quaterniond::Identity(), c, s);
  require(s.phase == TrackingState::Phase::Lost, "long loss resets");
  TrackingState static_state;
  Eigen::Vector3d static_v;
  for (int i = 0; i < 100; ++i) {
    double tm = i / 30.;
    Eigen::Quaterniond pose(
        Eigen::AngleAxisd(.15 * std::sin(tm), Eigen::Vector3d::UnitZ()));
    auto b = from_stable(p0, obj, pose, c);
    auto value = lesson04_tracking(
        {b}, start + std::chrono::microseconds(std::llround(tm * 1e6)), pose, c,
        static_state);
    if (value)
      static_v = value->velocity;
  }
  require(static_v.norm() < .01,
          "gimbal rotation does not create spurious target speed");
  require(predicted_corners(
              static_state, c,
              Eigen::Quaterniond(Eigen::AngleAxisd(.15 * std::sin(99 / 30.),
                                                   Eigen::Vector3d::UnitZ())))
              .has_value(),
          "future position is projectable");
  std::cout << "PASS " << checks
            << " checks: pose, units, rotation, transforms, timestamp, CV "
               "filtering, PSD, camera motion and loss lifecycle\n";
} catch (const std::exception &e) {
  std::cerr << "FAIL after " << checks << " checks: " << e.what() << '\n';
  return 1;
}
