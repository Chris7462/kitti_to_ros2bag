#pragma once

// C++ Standard Library
#include <vector>
#include <string>
#include <filesystem>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fs = std::filesystem;

struct CalibrationData
{
  std::array<double, 9> K;
  std::vector<double> D;
  std::array<double, 2> S_rect;
  std::array<double, 9> R_rect;
  std::array<double, 12> P_rect;
};

class MessageConverter
{
public:
  explicit MessageConverter(const rclcpp::Logger & logger);

  sensor_msgs::msg::Image convert_image_to_msg(const fs::path & file_path,
    const rclcpp::Time & timestamp, const std::string & encoding, const std::string & frame_id);

  sensor_msgs::msg::NavSatFix convert_oxts_to_gps_msg(
    const std::vector<std::string> & oxts_tokenized_array, const rclcpp::Time & timestamp);

  geometry_msgs::msg::TwistStamped convert_oxts_to_vel_msg(
    const std::vector<std::string> & oxts_tokenized_array, const rclcpp::Time & timestamp);

  sensor_msgs::msg::Imu convert_oxts_to_imu_msg(
    const std::vector<std::string> & oxts_tokenized_array, const rclcpp::Time & timestamp);

  sensor_msgs::msg::PointCloud2 convert_velo_to_msg(
    const fs::path & file_path, const rclcpp::Time & timestamp);

  sensor_msgs::msg::CameraInfo convert_calib_to_msg(
    const std::string & calib_file, const rclcpp::Time & timestamp,
    const std::string & id, const std::string & frame_id);

  std::vector<std::string> parse_file_data(const fs::path & file_path);

private:
  rclcpp::Logger logger_;
};
