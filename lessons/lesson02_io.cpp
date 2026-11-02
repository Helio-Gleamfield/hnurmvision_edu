#include "io_context.hpp"
#include "io_state.hpp"
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <climits>
#include <cmath>
#include <iostream>
#include <thread>
#include <utility>

namespace training::io::detail {
namespace {
double milliseconds(TimePoint newer, TimePoint older) {
  return std::chrono::duration<double, std::milli>(newer - older).count();
}
bool recent(TimePoint stamp, TimePoint now, double limit) {
  const double age = milliseconds(now, stamp);
  return age >= -5.0 && age <= limit;
}
}

cv::Mat decode_image(const sensor_msgs::msg::Image& msg) {
  int channels = 0, conversion = -1;
  if (msg.encoding == "bgr8") channels = 3;
  else if (msg.encoding == "rgb8") { channels = 3; conversion = cv::COLOR_RGB2BGR; }
  else if (msg.encoding == "mono8") { channels = 1; conversion = cv::COLOR_GRAY2BGR; }
  else if (msg.encoding == "rgba8") { channels = 4; conversion = cv::COLOR_RGBA2BGR; }
  else if (msg.encoding == "bgra8") { channels = 4; conversion = cv::COLOR_BGRA2BGR; }
  else return {};
  if (!msg.width || !msg.height || msg.width > static_cast<unsigned>(INT_MAX) ||
      msg.height > static_cast<unsigned>(INT_MAX)) return {};
  const auto row_bytes = static_cast<std::uint64_t>(msg.width) * channels;
  const auto bytes = static_cast<std::uint64_t>(msg.step) * msg.height;
  if (msg.step < row_bytes || bytes > msg.data.size()) return {};
  // 只在此处借用消息内存；返回前 clone/cvtColor，支持带行填充的消息。
  const cv::Mat view(static_cast<int>(msg.height), static_cast<int>(msg.width),
      CV_MAKETYPE(CV_8U, channels), const_cast<unsigned char*>(msg.data.data()), msg.step);
  if (conversion < 0) return view.clone();
  cv::Mat bgr;
  cv::cvtColor(view, bgr, conversion);
  return bgr;
}

Eigen::Quaterniond orientation(const Feedback& msg) {
  // prelude.rs: (x,y,z)_FLU = (-z,-x,y)_Bevy。
  // Bevy YXZ(yaw,-pitch,roll) 因而变成 FLU ZYX(yaw,pitch,-roll)。
  // 消息 pitch 为正时低头；这里是坐标旋转，不是 Command 的仰角。
  const double rad = kPi / 180.0;
  return Eigen::Quaterniond(Eigen::AngleAxisd(msg.yaw * rad, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(msg.pitch * rad, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-msg.roll * rad, Eigen::Vector3d::UnitX())).normalized();
}

InputBuffer::InputBuffer(std::int64_t ros_origin_ns, TimePoint steady_origin)
    : ros_origin_ns_(ros_origin_ns), steady_origin_(steady_origin) {}

std::optional<TimePoint> InputBuffer::convert(const builtin_interfaces::msg::Time& stamp) const {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
      (stamp.sec == 0 && stamp.nanosec == 0)) return std::nullopt;
  const auto ns = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  // 仿真器使用系统 ROS 时钟。固定锚点保留采集时间与传输延迟，绝不以到达时刻冒充采集时刻。
  return steady_origin_ + std::chrono::nanoseconds(ns - ros_origin_ns_);
}

void InputBuffer::receive_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
  const auto stamp = convert(msg->header.stamp);
  if (!stamp || *stamp <= newest_image_) return;
  if (image_ && *stamp <= *convert(image_->header.stamp)) return;
  image_ = std::move(msg);  // 仅保留最新帧，不堆积图像。
}

void InputBuffer::receive_feedback(const Feedback& msg) {
  const auto stamp = convert(msg.header.stamp);
  if (!stamp || !std::isfinite(msg.yaw) || !std::isfinite(msg.pitch) ||
      !std::isfinite(msg.roll) || msg.bullet_speed.data == 0) {
    history_.clear();
    fresh_input_ = false;
    return;
  }
  if (!history_.empty() && *stamp <= history_.back().time) return;
  latest_ = msg;
  history_.push_back({*stamp, orientation(msg), static_cast<double>(msg.bullet_speed.data)});
  while (history_.size() > 128 ||
      (!history_.empty() && milliseconds(*stamp, history_.front().time) > 1000))
    history_.pop_front();
}

std::optional<SensorData> InputBuffer::take(TimePoint now, const Config& config) {
  fresh_input_ = false;
  if (!image_ || history_.empty()) return std::nullopt;
  const auto stamp = *convert(image_->header.stamp);
  if (!recent(stamp, now, config.max_input_age_ms) ||
      !recent(history_.back().time, now, config.max_input_age_ms)) {
    image_.reset();
    if (history_.back().time > now + std::chrono::milliseconds(5)) history_.clear();
    return std::nullopt;
  }
  auto after = std::lower_bound(history_.begin(), history_.end(), stamp,
      [](const Sample& sample, TimePoint t) { return sample.time < t; });
  Sample aligned = after == history_.end() ? history_.back() : *after;
  if (after != history_.begin() && after != history_.end()) {
    const auto& before = *std::prev(after);
    const double gap = milliseconds(after->time, before.time);
    if (gap <= config.max_feedback_gap_ms) {
      const double fraction = milliseconds(stamp, before.time) / gap;
      aligned = {stamp, before.q.slerp(fraction, after->q).normalized(),
          fraction < 0.5 ? before.speed : after->speed};
    } else if (milliseconds(stamp, before.time) < milliseconds(after->time, stamp)) {
      aligned = before;
    }
  }
  // 一侧采样时使用最近姿态，时间差受限；等不到反馈时立即返回，保留最新图像等待下一轮。
  if (std::abs(milliseconds(stamp, aligned.time)) > config.max_feedback_gap_ms)
    return std::nullopt;
  auto message = std::move(image_);
  cv::Mat bgr = decode_image(*message);
  if (bgr.empty()) return std::nullopt;
  newest_image_ = stamp;
  input_time_ = stamp;
  fresh_input_ = true;
  ++frames_;
  return SensorData{std::move(bgr), stamp, aligned.q, aligned.speed};
}

Output InputBuffer::command(const Command& value, TimePoint now, const Config& config) const {
  Output msg;  // 停止默认角度、距离、控制和射击均为 0。
  msg.header.frame_id = "gimbal_link";
  if (!fresh_input_ || history_.empty() || !config.allow_control || !value.control ||
      latest_.work_mode.data != hnurm_interfaces::msg::WorkMode::AUTO_AIM ||
      !recent(input_time_, now, config.max_input_age_ms) ||
      !recent(history_.back().time, now, config.max_input_age_ms) ||
      !std::isfinite(value.yaw) || !std::isfinite(value.pitch) ||
      std::abs(value.pitch) > config.pitch_limit_rad) return msg;
  msg.target_state.data = value.shoot && config.allow_fire ?
      hnurm_interfaces::msg::TargetState::FIRE : hnurm_interfaces::msg::TargetState::CONVERGING;
  msg.yaw = static_cast<float>(std::remainder(value.yaw, 2 * kPi) * 180 / kPi);
  msg.pitch = static_cast<float>(-value.pitch * 180 / kPi);
  // 当前 Rust bridge 将 -1 解释为停止；未测距用 0，是否控制完全由 target_state 决定。
  msg.target_distance = 0;
  return msg;
}
}  // namespace training::io::detail

namespace training::io {
struct IoContext::Impl {
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr images;
  rclcpp::Subscription<detail::Feedback>::SharedPtr feedback;
  rclcpp::Publisher<detail::Output>::SharedPtr publisher;
  std::unique_ptr<detail::InputBuffer> input;
  TimePoint last_log{};
  std::uint64_t reads = 0;

  ~Impl() {
    if (context && context->is_valid()) context->shutdown("IO resources released");
  }
};

IoContext::IoContext(Config configuration) : config(std::move(configuration)) {}
IoContext::~IoContext() {
  try { close(); }
  catch (const std::exception& error) { std::cerr << "[IO Close] " << error.what() << '\n'; }
}

void IoContext::open() {
  if (impl_) return;
  auto state = std::make_unique<Impl>();
  state->context = std::make_shared<rclcpp::Context>();
  // 不安装 ROS 全局信号处理器，保留 main.cpp 的 Ctrl+C 退出流程。
  state->context->init(0, nullptr);
  state->node = std::make_shared<rclcpp::Node>("training_io",
      rclcpp::NodeOptions().context(state->context));
  const auto steady_origin = Clock::now();
  state->input = std::make_unique<detail::InputBuffer>(state->node->now().nanoseconds(), steady_origin);
  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
  auto* input = state->input.get();
  state->images = state->node->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", sensor_qos,
      [input](sensor_msgs::msg::Image::ConstSharedPtr msg) { input->receive_image(std::move(msg)); });
  state->feedback = state->node->create_subscription<detail::Feedback>(
      "/vision_receive_data", sensor_qos,
      [input](detail::Feedback::ConstSharedPtr msg) { input->receive_feedback(*msg); });
  // reliable 与仿真器 best-effort 订阅兼容，也能被默认 ros2 topic echo 订阅。
  state->publisher = state->node->create_publisher<detail::Output>(
      "/vision_send_data", rclcpp::QoS(5).reliable());
  rclcpp::ExecutorOptions options;
  options.context = state->context;
  state->executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(options);
  state->executor->add_node(state->node);
  impl_ = std::move(state);
  std::cout << "[IO Open] /training_io ready: /image_raw + /vision_receive_data -> /vision_send_data\n";
}

std::optional<SensorData> IoContext::read() {
  if (!impl_ || !impl_->context->is_valid()) return std::nullopt;
  auto& state = *impl_;
  // 单线程，回调不与 read/write 竞争；2 ms 预算不等待未来消息。
  state.executor->spin_some(std::chrono::milliseconds(2));
  const auto now = Clock::now();
  auto result = state.input->take(now, config);
  if (result && now - state.last_log >= std::chrono::seconds(1)) {
    std::cout << "yaw=" << state.input->yaw_deg() << " deg pitch=" << state.input->pitch_deg()
              << " deg bullet_speed=" << result->bullet_speed << " m/s\n";
    state.last_log = now;
  }
  if (++state.reads % 100 == 0)
    std::cout << "[IO Read] loops=" << state.reads << " frames=" << state.input->frames()
              << " input=" << (result ? "ready" : "waiting") << '\n';
  if (!result) std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return result;
}

void IoContext::write(const Command& command) {
  if (!impl_ || !impl_->context->is_valid()) return;
  auto msg = impl_->input->command(command, Clock::now(), config);
  msg.header.stamp = impl_->node->now();
  impl_->publisher->publish(msg);
}

void IoContext::close() {
  if (!impl_) return;
  if (impl_->context->is_valid()) {
    detail::Output stop;
    stop.header.stamp = impl_->node->now();
    stop.header.frame_id = "gimbal_link";
    impl_->publisher->publish(stop);
    // 最多等待 50 ms 让可靠订阅者确认停止；不无限等待已离线的进程。
    impl_->publisher->wait_for_all_acked(std::chrono::milliseconds(50));
    impl_->executor->cancel();
    impl_->executor->remove_node(impl_->node);
    impl_->context->shutdown("training IO closed");
  }
  impl_.reset();
  std::cout << "[IO Close] stop published; ROS2 resources released\n";
}
}  // namespace training::io
