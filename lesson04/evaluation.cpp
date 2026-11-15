#include "evaluation.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
namespace training {
namespace {
Eigen::Vector3d vec(const YAML::Node &n) {
  return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}
Eigen::Vector3d to_flu(const Eigen::Vector3d &b) {
  return {-b.z(), -b.x(), b.y()};
}
struct Plate {
  int id;
  Eigen::Vector3d camera, stable;
  std::array<Eigen::Vector3d, 4> corners;
};
struct Truth {
  std::int64_t stamp;
  std::vector<Plate> plates;
};
struct Measurement {
  std::int64_t stamp;
  PoseObservation pose;
  Eigen::Vector3d predicted;
  double horizon_ms;
  bool forecast;
  CameraCalibration calibration;
};
struct Forecast {
  std::int64_t issued, due;
  int id;
  Eigen::Vector3d position;
  double horizon_ms;
};
const Plate *find_plate(const Truth &p, int id) {
  for (const auto &v : p.plates)
    if (v.id == id)
      return &v;
  return nullptr;
}
std::array<Eigen::Vector3d, 4> ordered(const Plate &p,
                                       const CameraCalibration &c) {
  std::array<Eigen::Vector3d, 4> points = p.corners;
  auto pixel = [&](const auto &v) {
    return cv::Point2d(
        c.matrix.at<double>(0, 0) * v.x() / v.z() + c.matrix.at<double>(0, 2),
        c.matrix.at<double>(1, 1) * v.y() / v.z() + c.matrix.at<double>(1, 2));
  };
  cv::Point2d center;
  for (auto v : points)
    center += pixel(v) * .25;
  std::sort(points.begin(), points.end(), [&](const auto &a, const auto &b) {
    auto x = pixel(a) - center, y = pixel(b) - center;
    return std::atan2(x.y, x.x) < std::atan2(y.y, y.x);
  });
  auto start = std::min_element(points.begin(), points.end(),
                                [&](const auto &a, const auto &b) {
                                  auto x = pixel(a), y = pixel(b);
                                  return x.x + x.y < y.x + y.y;
                                });
  std::rotate(points.begin(), start, points.end());
  return points;
}
} // namespace
struct Lesson04Evaluation::Impl {
  std::shared_ptr<rclcpp::Context> context;
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  std::map<std::int64_t, Truth> truths;
  std::map<std::int64_t, Measurement> waiting;
  std::deque<Forecast> forecasts;
  EvaluationView view;
  std::vector<double> pose_errors, rot_errors, pred_errors;
  std::ofstream pose_csv, pred_csv;
  std::uint64_t rejected_truth = 0, rejected_forecasts = 0;
  Impl(const std::string &prefix) {
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    node = std::make_shared<rclcpp::Node>(
        "lesson04_evaluation", rclcpp::NodeOptions().context(context));
    rclcpp::ExecutorOptions options;
    options.context = context;
    executor =
        std::make_unique<rclcpp::executors::SingleThreadedExecutor>(options);
    executor->add_node(node);
    sub = node->create_subscription<std_msgs::msg::String>(
        "/lesson04/ground_truth", rclcpp::QoS(100),
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          try {
            auto root = YAML::Load(m->data);
            Truth t;
            t.stamp = root["stamp_sec"].as<std::int64_t>() * 1000000000LL +
                      root["stamp_nanosec"].as<std::int64_t>();
            auto origin = vec(root["gimbal_world_bevy"]);
            for (auto n : root["plates"]) {
              Plate p;
              p.id = n["id"].as<int>();
              p.camera = vec(n["center_camera"]);
              p.stable = to_flu(vec(n["center_world_bevy"]) - origin);
              int i = 0;
              for (auto v : n["corners_camera"])
                p.corners.at(i++) = vec(v);
              if (i == 4)
                t.plates.push_back(p);
            }
            truths[t.stamp] = std::move(t);
            while (truths.size() > 250)
              truths.erase(truths.begin());
          } catch (const std::exception &) {
            ++rejected_truth;
          }
        });
    if (!prefix.empty()) {
      pose_csv.open(prefix + "-poses.csv");
      pred_csv.open(prefix + "-forecasts.csv");
      pose_csv << "stamp_ns,truth_id,pnp_x_m,pnp_y_m,pnp_z_m,truth_x_m,truth_y_"
                  "m,truth_z_m,position_error_m,rotation_error_deg,stable_"
                  "error_m,reprojection_px\n";
      pred_csv << "issued_ns,due_ns,evaluated_ns,horizon_ms,truth_id,pred_x_m,"
                  "pred_y_m,pred_z_m,true_x_m,true_y_m,true_z_m,error_m,"
                  "interpolation_gap_ms\n";
      pose_csv << std::setprecision(12);
      pred_csv << std::setprecision(12);
    }
  }
  ~Impl() {
    executor->cancel();
    executor->remove_node(node);
    context->shutdown("lesson04 evaluation exit");
  }
  void reconcile() {
    for (auto it = waiting.begin(); it != waiting.end();) {
      auto truth = truths.find(it->first);
      if (truth == truths.end()) {
        if (!truths.empty() &&
            it->first < truths.rbegin()->first - 500000000LL) {
          ++view.unmatched_frames;
          it = waiting.erase(it);
        } else
          ++it;
        continue;
      }
      const auto &m = it->second;
      const Plate *best = nullptr;
      double distance = 1e9;
      cv::Point2f center;
      for (auto v : m.pose.armor.corners)
        center += v * .25f;
      for (auto &p : truth->second.plates) {
        if (p.camera.z() <= .1)
          continue;
        double u = m.calibration.matrix.at<double>(0, 0) * p.camera.x() /
                       p.camera.z() +
                   m.calibration.matrix.at<double>(0, 2),
               v = m.calibration.matrix.at<double>(1, 1) * p.camera.y() /
                       p.camera.z() +
                   m.calibration.matrix.at<double>(1, 2);
        double d = std::hypot(u - center.x, v - center.y);
        if (d < distance && d < 35) {
          distance = d;
          best = &p;
        }
      }
      if (best) {
        auto corners = ordered(*best, m.calibration);
        Eigen::Vector3d x = (corners[1] - corners[0]).normalized(),
                        y = (corners[3] - corners[0]).normalized();
        Eigen::Vector3d z = x.cross(y).normalized();
        y = z.cross(x).normalized();
        Eigen::Matrix3d gt;
        gt.col(0) = x;
        gt.col(1) = y;
        gt.col(2) = z;
        view.truth_available = true;
        view.true_camera = best->camera;
        view.true_stable = best->stable;
        view.pose_error_m =
            (Eigen::Vector3d(m.pose.tvec[0], m.pose.tvec[1], m.pose.tvec[2]) -
             best->camera)
                .norm();
        view.stable_error_m = (m.pose.position - best->stable).norm();
        view.rotation_error_deg =
            std::acos(std::clamp(
                ((m.pose.rotation_camera * gt.transpose()).trace() - 1) / 2,
                -1., 1.)) *
            180 / kPi;
        ++view.pose_samples;
        pose_errors.push_back(view.pose_error_m);
        rot_errors.push_back(view.rotation_error_deg);
        if (pose_csv)
          pose_csv << m.stamp << ',' << best->id << ',' << m.pose.tvec[0] << ','
                   << m.pose.tvec[1] << ',' << m.pose.tvec[2] << ','
                   << best->camera.x() << ',' << best->camera.y() << ','
                   << best->camera.z() << ',' << view.pose_error_m << ','
                   << view.rotation_error_deg << ',' << view.stable_error_m
                   << ',' << m.pose.reprojection_px << '\n';
        if (m.forecast)
          forecasts.push_back(
              {m.stamp,
               m.stamp + std::int64_t(std::llround(m.horizon_ms * 1e6)),
               best->id, m.predicted, m.horizon_ms});
      } else
        ++view.unmatched_frames;
      it = waiting.erase(it);
    }
    for (auto it = forecasts.begin(); it != forecasts.end();) {
      if (truths.empty() || truths.rbegin()->first < it->due) {
        ++it;
        continue;
      }
      auto hi = truths.lower_bound(it->due);
      if (hi == truths.end() || hi == truths.begin()) {
        ++rejected_forecasts;
        it = forecasts.erase(it);
        continue;
      }
      auto lo = std::prev(hi);
      double gap = double(hi->first - lo->first) / 1e9;
      auto a = find_plate(lo->second, it->id),
           b = find_plate(hi->second, it->id);
      if (!a || !b || gap > .10) {
        ++rejected_forecasts;
        it = forecasts.erase(it);
        continue;
      }
      double alpha =
          double(it->due - lo->first) / double(hi->first - lo->first);
      Eigen::Vector3d actual = a->stable + (b->stable - a->stable) * alpha;
      double error = (it->position - actual).norm();
      view.prediction_available = true;
      view.prediction_error_m = error;
      view.prediction_horizon_ms = it->horizon_ms;
      view.evaluation_lag_ms = double(hi->first - it->due) / 1e6;
      ++view.prediction_samples;
      pred_errors.push_back(error);
      if (pred_csv)
        pred_csv << it->issued << ',' << it->due << ',' << hi->first << ','
                 << it->horizon_ms << ',' << it->id << ',' << it->position.x()
                 << ',' << it->position.y() << ',' << it->position.z() << ','
                 << actual.x() << ',' << actual.y() << ',' << actual.z() << ','
                 << error << ',' << gap * 1000 << '\n';
      it = forecasts.erase(it);
    }
  }
};
Lesson04Evaluation::Lesson04Evaluation(const std::string &prefix)
    : impl_(std::make_unique<Impl>(prefix)) {}
Lesson04Evaluation::~Lesson04Evaluation() = default;
void Lesson04Evaluation::poll() {
  impl_->executor->spin_some(std::chrono::milliseconds(1));
  impl_->reconcile();
}
void Lesson04Evaluation::observe(const SensorData &in, const TrackingState &s,
                                 const CameraCalibration &c) {
  if (s.observation && in.acquisition_ros_ns > 0) {
    Measurement m;
    m.stamp = in.acquisition_ros_ns;
    m.pose = *s.observation;
    m.calibration = c;
    m.horizon_ms = s.parameters.prediction_ms;
    m.forecast = s.phase == TrackingState::Phase::Tracking;
    m.predicted = s.x.head<3>() + s.x.tail<3>() * (m.horizon_ms / 1000.);
    impl_->waiting[m.stamp] = m;
  }
  poll();
}
EvaluationView Lesson04Evaluation::view() const { return impl_->view; }
void Lesson04Evaluation::write_summary(const std::string &path) const {
  auto stats = [](std::vector<double> v) {
    YAML::Node n;
    n["count"] = v.size();
    if (!v.empty()) {
      std::sort(v.begin(), v.end());
      double sum = std::accumulate(v.begin(), v.end(), 0.);
      n["mean"] = sum / v.size();
      n["median"] = v[v.size() / 2];
      n["p95"] = v[std::min(v.size() - 1, size_t(.95 * v.size()))];
      n["max"] = v.back();
    }
    return n;
  };
  YAML::Node n;
  n["pose_error_m"] = stats(impl_->pose_errors);
  n["orientation_error_deg"] = stats(impl_->rot_errors);
  n["delayed_prediction_error_m"] = stats(impl_->pred_errors);
  n["unmatched_frames"] = impl_->view.unmatched_frames;
  n["invalid_truth_messages"] = impl_->rejected_truth;
  n["unevaluable_forecasts"] = impl_->rejected_forecasts;
  n["evaluation_method"] =
      "Exact image stamp; independent rendered marker geometry; wait for "
      "timestamp bracket at t+horizon; interpolate only after future frames "
      "arrive";
  std::ofstream(path) << n;
}
cv::Mat draw_tracking(const cv::Mat &image, const SensorData &in,
                      const TrackingState &s, const CameraCalibration &c,
                      const EvaluationView &e) {
  cv::Mat out = image.clone();
  int line = 32;
  auto text = [&](const std::string &str,
                  cv::Scalar color = cv::Scalar(255, 255, 255)) {
    cv::putText(out, str, {17, line}, cv::FONT_HERSHEY_SIMPLEX, .66, {0, 0, 0},
                4, cv::LINE_AA);
    cv::putText(out, str, {17, line}, cv::FONT_HERSHEY_SIMPLEX, .66, color, 2,
                cv::LINE_AA);
    line += 29;
  };
  text(std::string("Lesson04 ") + tracking_phase_name(s.phase) +
       cv::format("   horizon %.0f ms", s.parameters.prediction_ms));
  text("GREEN: detected armor    MAGENTA: predicted future armor",
       {255, 220, 120});
  if (s.observation) {
    auto &p = *s.observation;
    text(cv::format("PnP camera [m]  x=%.3f y=%.3f z=%.3f", p.tvec[0],
                    p.tvec[1], p.tvec[2]));
    text(cv::format("Reprojection %.2f px   velocity [m/s] %.3f %.3f %.3f",
                    p.reprojection_px, s.x(3), s.x(4), s.x(5)));
    cv::drawFrameAxes(out, c.matrix, c.distortion, p.rvec, p.tvec, .06, 2);
  }
  if (auto corners = predicted_corners(s, c, in.imu_q)) {
    std::vector<cv::Point> p;
    for (auto v : *corners)
      p.emplace_back(cvRound(v.x), cvRound(v.y));
    cv::polylines(out, p, true, {255, 0, 255}, 3, cv::LINE_AA);
  }
  if (e.truth_available) {
    text(cv::format("GT camera [m]   x=%.3f y=%.3f z=%.3f", e.true_camera.x(),
                    e.true_camera.y(), e.true_camera.z()),
         {255, 255, 0});
    text(cv::format("Pose error %.1f mm   rotation error %.2f deg",
                    e.pose_error_m * 1000, e.rotation_error_deg),
         {255, 255, 0});
    text(cv::format("Stable frame position error %.1f mm",
                    e.stable_error_m * 1000),
         {255, 255, 0});
  } else
    text("Ground truth: waiting for synchronized frame", {0, 190, 255});
  line = out.rows - 68;
  if (e.prediction_available) {
    text(cv::format("%.0f ms DELAYED prediction error: %.1f mm",
                    e.prediction_horizon_ms, e.prediction_error_m * 1000),
         {255, 80, 255});
    text(cv::format("Verified after future frame arrived; bracket lag %.1f ms; "
                    "samples %llu",
                    e.evaluation_lag_ms,
                    (unsigned long long)e.prediction_samples),
         {255, 180, 255});
  } else
    text("Delayed prediction error: WAITING for future observation",
         {0, 190, 255});
  return out;
}
} // namespace training
