#include "rclcpp/rclcpp.hpp"
#include "ORB2Ros.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto manager = std::make_shared<ORB2Ros>();
  rclcpp::spin(manager);
  rclcpp::shutdown();
  return 0;
}
