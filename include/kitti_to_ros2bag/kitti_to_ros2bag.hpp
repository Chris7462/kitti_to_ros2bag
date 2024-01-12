#pragma once

// C++ header
#include <vector>
#include <array>
#include <string>
#include <filesystem>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>


namespace fs = std::filesystem;

struct CalibrationData
{
  std::array<double, 9> K;
  std::vector<double> D;
  std::array<double, 2> S_rect;
  std::array<double, 9> R_rect;
  std::array<double, 12> P_rect;
};

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

  sensor_msgs::msg::Image convert_image_to_msg(
    const fs::path & file_path, const rclcpp::Time & timestamp,
    const std::string & encoding, const std::string & frame_id);

  std::vector<std::string> parse_file_data(const fs::path & file_path, std::string delimiter);

  sensor_msgs::msg::NavSatFix convert_oxts_to_gps_msg(
    const std::vector<std::string> & oxts_tokenized_array,
    const rclcpp::Time & timestamp);

  geometry_msgs::msg::TwistStamped convert_oxts_to_vel_msg(
    const std::vector<std::string> & oxts_tokenized_array,
    const rclcpp::Time & timestamp);

  sensor_msgs::msg::Imu convert_oxts_to_imu_msg(
    const std::vector<std::string> & oxts_tokenized_array,
    const rclcpp::Time & timestamp);

  sensor_msgs::msg::PointCloud2 convert_velo_to_msg(
    const fs::path & file_path, const rclcpp::Time & timestamp);

  sensor_msgs::msg::CameraInfo convert_calib_to_msg(
    const std::string & calib_file, const rclcpp::Time & timestamp,
    const std::string & id, const std::string & frame_id);

  size_t index_;
  size_t max_index_;

  fs::path kitti_path_;
  std::string data_folder_;
  std::string calib_folder_;
  std::vector<std::string> dirs_;
  std::vector<std::vector<std::string>> filenames_;
  std::vector<std::vector<rclcpp::Time>> timestamps_;
};
