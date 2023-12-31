#pragma once

#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>


class Kitti2BagNode : public rclcpp::Node
{
public:
  enum Type : uint8_t
  {
    oxts, // 0
    point_cloud,  // 1
    left_gray_image,  // 2
    right_gray_image, // 3
    left_color_image, // 4
    right_color_image,  // 5
    maxTypes
  };

  Kitti2BagNode();

private:
  void on_timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;

  void get_all_filenames();

  size_t index_;
  size_t max_index_;

  std::string oxts_path_;
  std::string point_cloud_path_;
  std::string left_gray_image_path_;
  std::string right_gray_image_path_;
  std::string left_color_image_path_;
  std::string right_color_image_path_;

  std::vector<std::string> oxts_filenams_;
  std::vector<std::string> point_cloud_filenams_;
  std::vector<std::string> left_gray_image_filenams_;
  std::vector<std::string> right_gray_image_filenams_;
  std::vector<std::string> left_color_image_filenams_;
  std::vector<std::string> right_color_image_filenams_;
};
