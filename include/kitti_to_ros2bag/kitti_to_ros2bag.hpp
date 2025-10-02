#pragma once

// C++ header
#include <vector>
#include <array>
#include <string>
#include <filesystem>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>


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
  void create_topics();
  void create_topic(const std::string & topic_name, const std::string & topic_type);

  template<typename T>
  void write_message(const T & msg, const std::string & topic_name, const rclcpp::Time & timestamp)
  {
    // Serialize the message
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<T> serializer;
    serializer.serialize_message(&msg, &serialized_msg);

    // Create a bag message
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_message->topic_name = topic_name;
    bag_message->recv_timestamp = timestamp.nanoseconds();

    // Get the serialized data
    auto & rcl_serialized_msg = serialized_msg.get_rcl_serialized_message();
    bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>();
    *bag_message->serialized_data = rcl_serialized_msg;

    // Write the message to the bag
    writer_->write(bag_message);
  }

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::TimerBase::SharedPtr timer_;

  void get_filenames();
  void get_all_timestamps();

  sensor_msgs::msg::Image convert_image_to_msg(
    const fs::path & file_path, const rclcpp::Time & timestamp,
    const std::string & encoding, const std::string & frame_id);

  std::vector<std::string> parse_file_data(const fs::path & file_path);

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
