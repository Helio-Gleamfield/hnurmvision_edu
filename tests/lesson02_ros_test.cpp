#include "io_context.hpp"
#include "io_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace training;
using namespace training::io;
using namespace training::io::detail;
using namespace std::chrono_literals;
int main() try {
  Config config; config.display = false;
  IoContext io(config);
  io.open(); io.open();  // Open/Close 均幂等，且支持重新连接。
  auto ctx = std::make_shared<rclcpp::Context>(); ctx->init(0,nullptr);
  auto peer = std::make_shared<rclcpp::Node>("lesson02_test_peer",rclcpp::NodeOptions().context(ctx));
  auto images = peer->create_publisher<sensor_msgs::msg::Image>("/image_raw",rclcpp::SensorDataQoS());
  auto feedbacks = peer->create_publisher<Feedback>("/vision_receive_data",rclcpp::SensorDataQoS());
  std::optional<Output> received;
  auto sub = peer->create_subscription<Output>("/vision_send_data",rclcpp::QoS(5),
      [&](Output::ConstSharedPtr m) { received = *m; });
  rclcpp::ExecutorOptions options; options.context = ctx;
  rclcpp::executors::SingleThreadedExecutor exec(options); exec.add_node(peer);
  auto expect = [](bool ok, const char* text) { if (!ok) throw std::runtime_error(text); };
  auto await = [&](auto predicate) {
    auto deadline = Clock::now()+3s;
    while (Clock::now()<deadline) {
      exec.spin_some(2ms);
      if (predicate()) return true;
      std::this_thread::sleep_for(5ms);
    }
    return false;
  };
  expect(await([&] { return images->get_subscription_count() && feedbacks->get_subscription_count() &&
                           sub->get_publisher_count(); }), "DDS discovery");
  auto start = Clock::now(); expect(!io.read(), "no data returns empty");
  expect(Clock::now()-start < 100ms, "no-data read bounded");
  auto frame = [&] {
    Feedback feedback; feedback.header.stamp = peer->now(); feedback.yaw = -90;
    feedback.pitch = 25; feedback.bullet_speed.data = 25; feedbacks->publish(feedback);
    sensor_msgs::msg::Image image; image.header = feedback.header;
    image.width = 1; image.height = 1; image.encoding = "rgb8"; image.step = 3; image.data = {1,2,3};
    images->publish(image);
    std::optional<SensorData> result;
    expect(await([&] { result=io.read(); return result.has_value(); }), "receive ROS image and feedback");
    expect(result->image.at<cv::Vec3b>(0,0) == cv::Vec3b(3,2,1) && result->bullet_speed == 25,
           "ROS SensorData contents");
  };
  frame();
  const Command command{true,true,-kPi/2,0.1};
  io.write(command);
  expect(await([&] { return received && received->target_state.data == 1; }), "ROS forbid-fire command");
  expect(received->header.frame_id == "gimbal_link" && received->header.stamp.sec > 0, "ROS output header");
  io.config.allow_control = false; received.reset(); io.write(command);
  expect(await([&] { return received && received->target_state.data == 0; }), "ROS disable control");
  io.config.allow_control = true; io.config.allow_fire = true; frame(); received.reset(); io.write(command);
  expect(await([&] { return received && received->target_state.data == 2; }), "ROS fire gate");
  // 同一图像不重复交付，下一轮应发送停止。
  expect(!io.read(), "ROS duplicate delivery"); received.reset(); io.write(command);
  expect(await([&] { return received && received->target_state.data == 0; }), "ROS missing input stop");
  frame(); io.write(command);
  expect(await([&] { return received && received->target_state.data == 2; }), "active before Close");
  received.reset(); io.close(); io.close();
  expect(await([&] { return received && received->target_state.data == 0; }), "Close stop delivered");
  io.open(); expect(!io.read(), "reopen clears old input"); io.close();
  exec.remove_node(peer); ctx->shutdown("test finished");
  std::cout << "PASS: ROS2 Open Read Write Close in-process integration (not simulator IPC acceptance)\n";
  return 0;
} catch (const std::exception& error) {
  std::cerr << "ROS2 integration failed: " << error.what() << '\n';
  return 1;
}
