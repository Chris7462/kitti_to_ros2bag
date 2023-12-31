#pragma once

#include <vector>
#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>


class Kitti2BagNode : public rclcpp::Node
{
public:
  Kitti2BagNode();

private:
  void on_timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;

  void get_filenames();
  void get_all_timestamps();

  size_t index_;
  size_t max_index_;

  std::string kitti_path_;
  std::vector<std::string> dirs_;
  std::vector<std::string> filenames_;
  std::vector<std::vector<rclcpp::Time>> timestamp_;
};
