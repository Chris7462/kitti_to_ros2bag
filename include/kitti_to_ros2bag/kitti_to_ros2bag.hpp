#pragma once

// C++
#include <vector>
#include <string>
#include <filesystem>
#include <memory>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// local
#include "kitti_to_ros2bag/data_loader.hpp"
#include "kitti_to_ros2bag/message_converter.hpp"
#include "kitti_to_ros2bag/bag_writer.hpp"

namespace fs = std::filesystem;

class Kitti2BagNode : public rclcpp::Node
{
public:
  Kitti2BagNode();
  ~Kitti2BagNode() = default;

private:
  void on_timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;

  // Helper objects
  std::unique_ptr<DataLoader> data_loader_;
  std::unique_ptr<MessageConverter> message_converter_;
  std::unique_ptr<BagWriter> bag_writer_;

  size_t index_;
  size_t max_index_;

  fs::path kitti_path_;
  std::string data_folder_;
  std::string calib_folder_;
  std::vector<std::string> dirs_;
  std::vector<std::vector<std::string>> filenames_;
  std::vector<std::vector<rclcpp::Time>> timestamps_;
};
