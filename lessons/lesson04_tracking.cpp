#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "lessons.hpp"
#include "pose_tracking.hpp"
namespace training {
namespace {
bool calibration_valid(const CameraCalibration &c,
                       const Eigen::Quaterniond &q) {
  return c.matrix.rows == 3 && c.matrix.cols == 3 &&
         c.matrix.type() == CV_64F && cv::checkRange(c.matrix) &&
         c.matrix.at<double>(0, 0) > 0 && c.matrix.at<double>(1, 1) > 0 &&
         c.camera_to_gimbal.allFinite() &&
         (c.camera_to_gimbal.transpose() * c.camera_to_gimbal -
          Eigen::Matrix3d::Identity())
                 .norm() < 1e-3 &&
         c.camera_to_gimbal.determinant() > .99 &&
         c.camera_to_gimbal_translation.allFinite() && q.coeffs().allFinite() &&
         q.norm() > .5 && q.norm() < 1.5;
}
Eigen::Matrix3d eigen_rotation(const cv::Mat &r) {
  Eigen::Matrix3d m;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      m(i, j) = r.at<double>(i, j);
  return m;
}
void start_track(TrackingState &s, const PoseObservation &p, TimePoint now) {
  s.initialized = true;
  s.phase = TrackingState::Phase::Detecting;
  s.hits = 1;
  ++s.generation;
  s.timestamp = s.last_observed = now;
  s.x.setZero();
  s.x.head<3>() = p.position;
  s.covariance.setIdentity();
  s.covariance.topLeftCorner<3, 3>() *= .01;
  s.observation = p;
  s.diagnostic = "confirming observations";
}
} // namespace
std::vector<cv::Point3d> armor_object_points(bool large,
                                             const TrackingParameters &p) {
  double w = (large ? p.large_width_m : p.small_width_m) / 2,
         h = p.height_m / 2;
  return {{-w, -h, 0}, {w, -h, 0}, {w, h, 0}, {-w, h, 0}};
}
std::optional<PoseObservation> solve_armor_pose(const Armor &a,
                                                const CameraCalibration &c,
                                                const Eigen::Quaterniond &q,
                                                const TrackingParameters &p) {
  if (!calibration_valid(c, q) || !std::isfinite(a.confidence) ||
      p.height_m <= 0 || p.small_width_m <= 0 || p.large_width_m <= 0)
    return {};
  for (auto v : a.corners)
    if (!std::isfinite(v.x) || !std::isfinite(v.y))
      return {};
  std::vector<cv::Point2f> image(a.corners.begin(), a.corners.end());
  double area = std::abs(cv::contourArea(image));
  if (area < 8 || !cv::isContourConvex(image))
    return {};
  std::vector<cv::Mat> rvecs, tvecs;
  auto points = armor_object_points(a.large_armor, p);
  try {
    cv::solvePnPGeneric(points, image, c.matrix, c.distortion, rvecs, tvecs,
                        false, cv::SOLVEPNP_IPPE);
    std::optional<PoseObservation> best;
    for (size_t k = 0; k < rvecs.size(); ++k) {
      auto rv = rvecs[k].clone(), tv = tvecs[k].clone();
      if (!cv::checkRange(rv) || !cv::checkRange(tv))
        continue;
      // Refine each valid planar hypothesis; reject all behind-camera
      // solutions.
      cv::solvePnPRefineLM(points, image, c.matrix, c.distortion, rv, tv);
      cv::Mat rotation;
      cv::Rodrigues(rv, rotation);
      Eigen::Matrix3d R = eigen_rotation(rotation);
      Eigen::Vector3d t(tv.at<double>(0), tv.at<double>(1), tv.at<double>(2));
      if (!t.allFinite() || t.z() < .1 || t.norm() > 80)
        continue;
      bool front = true;
      for (auto v : points)
        if ((R * Eigen::Vector3d(v.x, v.y, v.z) + t).z() <= 0)
          front = false;
      if (!front)
        continue;
      std::vector<cv::Point2d> reproj;
      cv::projectPoints(points, rv, tv, c.matrix, c.distortion, reproj);
      double sum = 0;
      for (int j = 0; j < 4; ++j)
        sum += cv::norm(reproj[j] - cv::Point2d(image[j])) *
               cv::norm(reproj[j] - cv::Point2d(image[j]));
      double error = std::sqrt(sum / 4);
      if (!std::isfinite(error) || error > p.max_reprojection_px)
        continue;
      if (p.use_tilt_prior) {
        const Eigen::Matrix3d C =
            q.normalized().toRotationMatrix() * c.camera_to_gimbal;
        Eigen::Vector3d axis = (C * R).col(0);
        double yaw0 = std::atan2(axis.y(), axis.x()),
               tilt = p.armor_tilt_deg * kPi / 180;
        std::vector<cv::Point2f> normalized;
        cv::undistortPoints(image, normalized, c.matrix, c.distortion);
        double constrained_error = 1e9;
        Eigen::Matrix3d constrained_R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d constrained_t = Eigen::Vector3d::Zero();
        for (int step = -30; step <= 30; ++step) {
          double yaw = yaw0 + step * kPi / 180;
          Eigen::Matrix3d stable;
          stable.col(0) = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0);
          stable.col(1) =
              Eigen::Vector3d(std::sin(yaw) * std::sin(tilt),
                              -std::cos(yaw) * std::sin(tilt), -std::cos(tilt));
          stable.col(2) = stable.col(0).cross(stable.col(1));
          Eigen::Matrix3d candidate = C.transpose() * stable;
          Eigen::Matrix<double, 8, 3> M;
          Eigen::Matrix<double, 8, 1> b;
          for (int j = 0; j < 4; ++j) {
            Eigen::Vector3d X =
                candidate * Eigen::Vector3d(points[j].x, points[j].y, 0);
            double u = normalized[j].x, v = normalized[j].y;
            M.row(2 * j) << 1, 0, -u;
            M.row(2 * j + 1) << 0, 1, -v;
            b(2 * j) = u * X.z() - X.x();
            b(2 * j + 1) = v * X.z() - X.y();
          }
          Eigen::Vector3d candidate_t =
              (M.transpose() * M).ldlt().solve(M.transpose() * b);
          if (candidate_t.z() < .1)
            continue;
          cv::Mat cm(3, 3, CV_64F);
          for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
              cm.at<double>(i, j) = candidate(i, j);
          cv::Vec3d cr;
          cv::Rodrigues(cm, cr);
          std::vector<cv::Point2d> cp;
          cv::projectPoints(
              points, cr,
              cv::Vec3d(candidate_t.x(), candidate_t.y(), candidate_t.z()),
              c.matrix, c.distortion, cp);
          double cost = 0;
          for (int j = 0; j < 4; ++j)
            cost += std::pow(cv::norm(cp[j] - cv::Point2d(image[j])), 2);
          cost = std::sqrt(cost / 4);
          if (cost < constrained_error) {
            constrained_error = cost;
            constrained_R = candidate;
            constrained_t = candidate_t;
          }
        }
        if (constrained_error <= std::min(p.max_reprojection_px, error + .75) &&
            (constrained_t - t).norm() < .3) {
          R = constrained_R;
          t = constrained_t;
          error = constrained_error;
          cv::Mat rr(3, 3, CV_64F);
          for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
              rr.at<double>(i, j) = R(i, j);
          cv::Rodrigues(rr, rv);
          tv = (cv::Mat_<double>(3, 1) << t.x(), t.y(), t.z());
        }
      }
      PoseObservation obs;
      obs.armor = a;
      obs.rvec =
          cv::Vec3d(rv.at<double>(0), rv.at<double>(1), rv.at<double>(2));
      obs.tvec =
          cv::Vec3d(tv.at<double>(0), tv.at<double>(1), tv.at<double>(2));
      obs.rotation_camera = R;
      obs.position = q.normalized() *
                     (c.camera_to_gimbal * t + c.camera_to_gimbal_translation);
      obs.reprojection_px = error;
      obs.image_area = area;
      if (!best || error < best->reprojection_px)
        best = obs;
    }
    return best;
  } catch (const cv::Exception &) {
    return {};
  }
}
std::optional<Target> lesson04_tracking(const std::vector<Armor> &armors,
                                        TimePoint now,
                                        const Eigen::Quaterniond &q,
                                        const CameraCalibration &c,
                                        TrackingState &s) {
  s.observation.reset();
  if (!calibration_valid(c, q)) {
    s.diagnostic = "invalid calibration or IMU";
    return {};
  }
  if (s.initialized && now <= s.timestamp) {
    s.diagnostic = "non-increasing image timestamp";
    return {};
  }
  std::vector<PoseObservation> observations;
  for (const auto &a : armors)
    if (auto p = solve_armor_pose(a, c, q, s.parameters))
      observations.push_back(*p);
  double dt = s.initialized
                  ? std::chrono::duration<double>(now - s.timestamp).count()
                  : 0;
  if (s.initialized && dt > s.parameters.reset_gap_s) {
    s.initialized = false;
    s.phase = TrackingState::Phase::Lost;
    s.hits = 0;
  }
  if (!s.initialized) {
    if (observations.empty()) {
      s.diagnostic = "no valid PnP observation";
      return {};
    }
    auto score = [&](const PoseObservation &p) {
      cv::Point2f center;
      for (auto v : p.armor.corners)
        center += v * .25f;
      double cx = c.matrix.at<double>(0, 2), cy = c.matrix.at<double>(1, 2);
      return .5 * std::hypot((center.x - cx) / c.matrix.at<double>(0, 0),
                             (center.y - cy) / c.matrix.at<double>(1, 1)) +
             .1 * p.reprojection_px - p.armor.confidence -
             .03 * std::log(p.image_area);
    };
    auto best = std::min_element(
        observations.begin(), observations.end(),
        [&](const auto &a, const auto &b) { return score(a) < score(b); });
    start_track(s, *best, now);
    return {};
  }
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity(),
                              Q = Eigen::Matrix<double, 6, 6>::Zero();
  F.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * dt;
  double variance = s.parameters.acceleration_sigma_mps2 *
                    s.parameters.acceleration_sigma_mps2;
  for (int k = 0; k < 3; ++k) {
    Q(k, k) = variance * dt * dt * dt * dt / 4;
    Q(k, k + 3) = Q(k + 3, k) = variance * dt * dt * dt / 2;
    Q(k + 3, k + 3) = variance * dt * dt;
  }
  s.x = F * s.x;
  s.covariance = F * s.covariance * F.transpose() + Q;
  s.timestamp = now;
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H.leftCols<3>().setIdentity();
  const auto R = Eigen::Matrix3d::Identity() *
                 std::pow(s.parameters.measurement_sigma_m, 2);
  Eigen::Matrix3d S = H * s.covariance * H.transpose() + R;
  auto solve = S.ldlt();
  double best_distance = std::numeric_limits<double>::infinity();
  const PoseObservation *match = nullptr;
  for (const auto &p : observations) {
    auto d = (p.position - s.x.head<3>()).eval();
    double mahal = d.dot(solve.solve(d));
    if (d.norm() < s.parameters.match_distance_m && mahal < 16.3 &&
        mahal < best_distance) {
      best_distance = mahal;
      match = &p;
    }
  }
  if (!match) {
    double age = std::chrono::duration<double>(now - s.last_observed).count();
    s.phase = TrackingState::Phase::TempLost;
    s.diagnostic = "observation rejected or missed";
    if (age > s.parameters.lost_timeout_s) {
      s.initialized = false;
      s.phase = TrackingState::Phase::Lost;
      s.hits = 0;
    }
    return {};
  }
  Eigen::Matrix<double, 6, 3> K =
      s.covariance * H.transpose() * solve.solve(Eigen::Matrix3d::Identity());
  s.x += K * (match->position - H * s.x);
  // Joseph form preserves covariance symmetry and positive semidefiniteness.
  Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity(),
                              A = I - K * H;
  s.covariance = A * s.covariance * A.transpose() + K * R * K.transpose();
  s.covariance = (s.covariance + s.covariance.transpose()).eval() * .5;
  s.observation = *match;
  s.last_observed = now;
  ++s.hits;
  s.phase = s.hits >= s.parameters.confirm_frames
                ? TrackingState::Phase::Tracking
                : TrackingState::Phase::Detecting;
  s.diagnostic = "matched";
  if (s.phase != TrackingState::Phase::Tracking)
    return {};
  return Target{s.x.head<3>(), s.x.tail<3>()};
}
std::optional<std::array<cv::Point2f, 4>>
predicted_corners(const TrackingState &s, const CameraCalibration &c,
                  const Eigen::Quaterniond &q) {
  if (!s.observation || s.phase != TrackingState::Phase::Tracking ||
      !calibration_valid(c, q))
    return {};
  Eigen::Vector3d future =
      s.x.head<3>() + s.x.tail<3>() * (s.parameters.prediction_ms / 1000.);
  Eigen::Vector3d t =
      c.camera_to_gimbal.transpose() *
      (q.normalized().conjugate() * future - c.camera_to_gimbal_translation);
  if (t.z() <= .1)
    return {};
  cv::Vec3d tv(t.x(), t.y(), t.z());
  std::vector<cv::Point2d> result;
  cv::projectPoints(
      armor_object_points(s.observation->armor.large_armor, s.parameters),
      s.observation->rvec, tv, c.matrix, c.distortion, result);
  std::array<cv::Point2f, 4> a;
  std::copy(result.begin(), result.end(), a.begin());
  return a;
}
const char *tracking_phase_name(TrackingState::Phase p) {
  switch (p) {
  case TrackingState::Phase::Lost:
    return "LOST";
  case TrackingState::Phase::Detecting:
    return "DETECTING";
  case TrackingState::Phase::Tracking:
    return "TRACKING";
  case TrackingState::Phase::TempLost:
    return "TEMP_LOST";
  }
  return "UNKNOWN";
}
} // namespace training
