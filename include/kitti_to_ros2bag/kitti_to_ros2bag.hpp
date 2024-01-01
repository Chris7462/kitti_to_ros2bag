#pragma once

#include <vector>
#include <string>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>


namespace fs = std::filesystem;

class Kitti2BagNode : public rclcpp::Node
{
public:
  Kitti2BagNode();

private:
  void on_timer_callback();

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::TimerBase::SharedPtr timer_;

  void get_filenames();
  void get_all_timestamps();

  size_t index_;
  size_t max_index_;

  fs::path kitti_path_;
  std::vector<std::string> dirs_;
  std::vector<std::vector<std::string>> filenames_;
  std::vector<std::vector<rclcpp::Time>> timestamps_;
};
