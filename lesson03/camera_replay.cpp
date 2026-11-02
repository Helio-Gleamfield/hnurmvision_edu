// Replay recorded camera video through the same ROS image/feedback interface.
#include <chrono>
#include <hnurm_interfaces/msg/vision_recv_data.hpp>
#include <iostream>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
int main(int argc, char** argv) try {
  if (argc != 2) throw std::runtime_error("camera_replay INPUT_VIDEO");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lesson03_recorded_camera_replay");
  auto images = node->create_publisher<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS());
  auto feedback = node->create_publisher<hnurm_interfaces::msg::VisionRecvData>(
      "/vision_receive_data", rclcpp::SensorDataQoS());
  cv::VideoCapture video(argv[1]);
  if (!video.isOpened()) throw std::runtime_error("Cannot open video");
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (rclcpp::ok() && images->get_subscription_count() == 0) {
    if (std::chrono::steady_clock::now() > deadline)
      throw std::runtime_error("No image subscriber");
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  cv::Mat frame;
  int count = 0;
  while (rclcpp::ok() && video.read(frame)) {
    hnurm_interfaces::msg::VisionRecvData f;
    f.header.stamp = node->now();
    f.yaw = -90;
    f.pitch = 0;
    f.bullet_speed.data = 25;
    feedback->publish(f);
    sensor_msgs::msg::Image image;
    image.header = f.header;
    image.encoding = "bgr8";
    image.width = frame.cols;
    image.height = frame.rows;
    image.step = frame.cols * 3;
    image.data.assign(frame.data, frame.data + frame.total() * 3);
    images->publish(image);
    rclcpp::spin_some(node);
    ++count;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  rclcpp::shutdown();
  std::cout << "Recorded camera frames published: " << count << '\n';
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
