#include "io_state.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace training;
using namespace training::io::detail;
using namespace std::chrono_literals;
namespace {
unsigned checks = 0;
void check(bool value, const char* reason) {
  ++checks;
  if (!value) throw std::runtime_error(reason);
}
bool near(double a, double b, double tolerance = 1e-6) { return std::abs(a - b) < tolerance; }
const TimePoint origin{100s};
builtin_interfaces::msg::Time stamp(int ms) {
  builtin_interfaces::msg::Time t;
  t.sec = 1000 + ms / 1000;
  t.nanosec = (ms % 1000) * 1000000;
  return t;
}
auto image(int ms = 90) {
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->header.stamp = stamp(ms);
  msg->width = 2; msg->height = 2; msg->step = 8; msg->encoding = "rgb8";
  msg->data = {1,2,3, 4,5,6, 99,99, 7,8,9, 10,11,12, 99,99};
  return msg;
}
Feedback feedback(int ms = 90, float yaw = 0) {
  Feedback msg;
  msg.header.stamp = stamp(ms);
  msg.bullet_speed.data = 25;
  msg.yaw = yaw;
  return msg;
}
InputBuffer ready(const Config& config = {}) {
  InputBuffer state(1000000000000LL, origin);
  state.receive_feedback(feedback()); state.receive_image(image());
  check(state.take(origin + 100ms, config).has_value(), "valid input");
  return state;
}

void pixels() {
  auto msg = image();
  auto bgr = decode_image(*msg);
  check(bgr.type() == CV_8UC3 && bgr.rows == 2 && bgr.cols == 2, "BGR type/dimensions");
  check(bgr.at<cv::Vec3b>(1, 0) == cv::Vec3b(9,8,7), "RGB conversion and padded rows");
  msg->data.assign(16, 0);
  check(bgr.at<cv::Vec3b>(0,0) == cv::Vec3b(3,2,1), "independent RGB memory");
  msg = image(); msg->encoding = "bgr8";
  auto clone = decode_image(*msg); msg->data.clear();
  check(clone.at<cv::Vec3b>(0,0) == cv::Vec3b(1,2,3), "independent BGR memory");
  msg = image(); msg->encoding = "mono8"; msg->width = 1; msg->height = 1;
  msg->step = 1; msg->data = {42};
  check(decode_image(*msg).at<cv::Vec3b>(0,0) == cv::Vec3b(42,42,42), "mono8");
  msg->encoding = "rgba8"; msg->step = 4; msg->data = {1,2,3,4};
  check(decode_image(*msg).at<cv::Vec3b>(0,0) == cv::Vec3b(3,2,1), "rgba8");
  msg->encoding = "bgra8";
  check(decode_image(*msg).at<cv::Vec3b>(0,0) == cv::Vec3b(1,2,3), "bgra8");
  msg->encoding = "16UC1"; check(decode_image(*msg).empty(), "unsupported encoding");
  msg = image(); msg->step = 5; check(decode_image(*msg).empty(), "short stride");
  msg = image(); msg->data.resize(15); check(decode_image(*msg).empty(), "truncated bytes");
  msg = image(); msg->width = 0; check(decode_image(*msg).empty(), "zero width");
  msg = image(); msg->width = UINT32_MAX; check(decode_image(*msg).empty(), "oversized width");
}

void rotations() {
  auto msg = feedback(90, 90);
  check((orientation(msg) * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitY()), "yaw left");
  msg.yaw = 0; msg.pitch = 30;
  check(near((orientation(msg) * Eigen::Vector3d::UnitX()).z(), -0.5), "wire pitch nose down");
  msg.pitch = 0; msg.roll = 90;
  check((orientation(msg) * Eigen::Vector3d::UnitY()).isApprox(-Eigen::Vector3d::UnitZ()), "wire roll sign");
  Eigen::Matrix3d basis;
  basis << 0,0,-1, -1,0,0, 0,1,0;
  for (const auto angles : {Eigen::Vector3d(-90,25,0), Eigen::Vector3d(33,-21,15),
                            Eigen::Vector3d(179,44,-37)}) {
    msg.yaw = angles.x(); msg.pitch = angles.y(); msg.roll = angles.z();
    const double rad = kPi / 180;
    const Eigen::Matrix3d bevy = (Eigen::AngleAxisd(angles.x()*rad, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-angles.y()*rad, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(angles.z()*rad, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    check(orientation(msg).toRotationMatrix().isApprox(basis * bevy * basis.transpose(), 1e-6),
          "orientation agrees with actual simulator basis / YXZ");
  }
}

void timing() {
  Config config;
  InputBuffer state(1000000000000LL, origin);
  check(!state.take(origin+100ms, config), "missing input");
  state.receive_image(image());
  check(!state.take(origin+100ms, config), "image waits for feedback");
  state.receive_feedback(feedback(80,179)); state.receive_feedback(feedback(100,-179));
  auto frame = state.take(origin+110ms, config);
  check(frame && frame->timestamp == origin+90ms, "capture timestamp retains 20ms latency");
  check(frame && near(frame->bullet_speed,25), "speed in m/s");
  check(frame && (frame->imu_q * Eigen::Vector3d::UnitX()).x() < -0.999, "slerp shortest arc across +/-180");
  check(!state.take(origin+110ms, config), "consume once");
  state.receive_image(image()); check(!state.take(origin+110ms, config), "duplicate image ignored");
  state.receive_image(image(85)); check(!state.take(origin+110ms, config), "out of order image ignored");
  state.receive_image(image(95)); check(state.take(origin+110ms, config).has_value(), "next new frame");
  state.receive_image(image(400)); check(!state.take(origin+110ms, config), "future image rejected");
  state.receive_image(image(105)); check(state.take(origin+110ms, config).has_value(), "recover from future image");
  auto old = ready(); old.receive_image(image(100));
  check(!old.take(origin+400ms, config), "stale image and feedback");
  InputBuffer gap(1000000000000LL, origin);
  gap.receive_feedback(feedback(10)); gap.receive_image(image(100));
  check(!gap.take(origin+110ms, config), "feedback gap bounded");
  gap.receive_feedback(feedback(110));
  check(gap.take(origin+115ms, config).has_value(), "late feedback completes pending input");
  auto bad = feedback(120); bad.yaw = std::numeric_limits<float>::quiet_NaN();
  gap.receive_feedback(bad); gap.receive_image(image(120));
  check(!gap.take(origin+125ms, config), "invalid feedback revokes input");
  gap.receive_feedback(feedback(400)); gap.receive_image(image(125));
  check(!gap.take(origin+130ms, config), "future feedback rejected");
  gap.receive_feedback(feedback(130)); gap.receive_image(image(130));
  check(gap.take(origin+135ms, config).has_value(), "recover from future feedback");
  auto zero = image(140); zero->header.stamp = builtin_interfaces::msg::Time(); gap.receive_image(zero);
  check(!gap.take(origin+145ms, config), "zero timestamp rejected");
}

void gates() {
  Config config;
  auto state = ready();
  const Command shoot{true,true,5*kPi/2,0.2};
  auto out = state.command(shoot,origin+100ms,config);
  check(out.target_state.data == 1 && near(out.yaw,90) && near(out.pitch,-0.2*180/kPi,1e-5),
        "forbid fire + rad to deg + pitch sign + yaw wrap");
  check(out.header.frame_id == "gimbal_link" && out.target_distance != -1, "bridge-compatible active command");
  config.allow_fire = true;
  check(state.command(shoot,origin+100ms,config).target_state.data == 2, "fire permitted explicitly");
  auto command = shoot; command.shoot = false;
  check(state.command(command,origin+100ms,config).target_state.data == 1, "track only");
  command.control = false;
  auto stop = state.command(command,origin+100ms,config);
  check(stop.target_state.data == 0 && stop.yaw == 0 && stop.pitch == 0 && stop.target_distance == 0,
        "control false sends fully zero stop");
  config.allow_control = false;
  check(state.command(shoot,origin+100ms,config).target_state.data == 0, "forbid all control config");
  config.allow_control = true;
  command = shoot; command.pitch = config.pitch_limit_rad + 0.001;
  check(state.command(command,origin+100ms,config).target_state.data == 0, "pitch limit");
  command = shoot; command.yaw = std::numeric_limits<double>::infinity();
  check(state.command(command,origin+100ms,config).target_state.data == 0, "infinite yaw");
  command = shoot; command.pitch = std::numeric_limits<double>::quiet_NaN();
  check(state.command(command,origin+100ms,config).target_state.data == 0, "NaN pitch");
  check(state.command(shoot,origin+400ms,config).target_state.data == 0, "write checks aging too");
  auto mode = feedback(100); mode.work_mode.data = 1; state.receive_feedback(mode);
  check(state.command(shoot,origin+100ms,config).target_state.data == 0, "non-auto-aim stops immediately");
  state = ready(); check(!state.take(origin+100ms,config), "no fresh frame on next iteration");
  check(state.command(shoot,origin+100ms,config).target_state.data == 0, "no fresh frame stops output");
  state = ready(); auto invalid = feedback(100); invalid.bullet_speed.data = 0;
  state.receive_feedback(invalid);
  check(state.command(shoot,origin+100ms,config).target_state.data == 0, "invalid bullet speed stops");
}
}
int main() try {
  pixels(); rotations(); timing(); gates();
  std::cout << "PASS: " << checks << " IO contract checks (no DDS/window required)\n";
  return 0;
} catch (const std::exception& error) {
  std::cerr << "FAIL after " << checks << " checks: " << error.what() << '\n';
  return 1;
}
