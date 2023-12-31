#include "kitti_to_ros2bag/kitti_to_ros2bag.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Kitti2BagNode>());
  rclcpp::shutdown();

  return 0;
}
