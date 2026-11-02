#pragma once
#include "config.hpp"
#include <hnurm_interfaces/msg/vision_recv_data.hpp>
#include <hnurm_interfaces/msg/vision_send_data.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cstdint>
#include <deque>
#include <optional>

namespace training::io::detail {
using Feedback = hnurm_interfaces::msg::VisionRecvData;
using Output = hnurm_interfaces::msg::VisionSendData;

// 与 ROS 调度解耦，可用已知时间戳验证内存、时间对齐、坐标及门控。
cv::Mat decode_image(const sensor_msgs::msg::Image& message);
Eigen::Quaterniond orientation(const Feedback& message);

class InputBuffer {
 public:
  InputBuffer(std::int64_t ros_origin_ns, TimePoint steady_origin);
  void receive_image(sensor_msgs::msg::Image::ConstSharedPtr message);
  void receive_feedback(const Feedback& message);
  std::optional<SensorData> take(TimePoint now, const Config& config);
  Output command(const Command& value, TimePoint now, const Config& config) const;
  bool has_fresh_input() const { return fresh_input_; }
  double yaw_deg() const { return latest_.yaw; }
  double pitch_deg() const { return latest_.pitch; }
  std::uint64_t frames() const { return frames_; }

 private:
  struct Sample { TimePoint time; Eigen::Quaterniond q; double speed; };
  std::optional<TimePoint> convert(const builtin_interfaces::msg::Time& stamp) const;
  std::int64_t ros_origin_ns_;
  TimePoint steady_origin_;
  sensor_msgs::msg::Image::ConstSharedPtr image_;
  TimePoint newest_image_{};
  TimePoint input_time_{};
  Feedback latest_{};
  std::deque<Sample> history_;
  bool fresh_input_ = false;
  std::uint64_t frames_ = 0;
};
}  // namespace training::io::detail
