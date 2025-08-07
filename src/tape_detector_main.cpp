#include "tape_detector_ros2/tape_detector_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<tape_detector_ros2::TapeDetectorNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}