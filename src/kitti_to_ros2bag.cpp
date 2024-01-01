#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgcodecs.hpp>

#include "kitti_to_ros2bag/kitti_to_ros2bag.hpp"


using namespace std::chrono_literals;


Kitti2BagNode::Kitti2BagNode()
: Node("kitti2bag_node"), index_{0}
{
  declare_parameter("kitti_path", rclcpp::PARAMETER_STRING);
  kitti_path_ = get_parameter("kitti_path").as_string();

  declare_parameter("dirs", rclcpp::PARAMETER_STRING_ARRAY);
  dirs_ = get_parameter("dirs").as_string_array();

  declare_parameter("output_bag_name", rclcpp::PARAMETER_STRING);
  std::string output_bag_name = get_parameter("output_bag_name").as_string();

  get_filenames();
  get_all_timestamps();

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(output_bag_name);

  // create topic for the bag
  for (auto & dir : dirs_) {
    if (dir == "image_00") {
      writer_->create_topic({"kitti/camera_gray_left", "sensor_msgs/msg/Image", rmw_get_serialization_format(), ""});
//  } else if (dir == "image_01") {
//    writer_->create_topic({"kitti/camera_gray_right", "sensor_msgs/msg/Image", rmw_get_serialization_format(), ""});
//  } else if (dir == "image_02") {
//    writer_->create_topic({"kitti/camera_color_left", "sensor_msgs/msg/Image", rmw_get_serialization_format(), ""});
//  } else if (dir == "image_03") {
//    writer_->create_topic({"kitti/camera_color_right", "sensor_msgs/msg/Image", rmw_get_serialization_format(), ""});
//  } else if (dir == "oxts") {
//    writer_->create_topic({"kitti/oxts/imu", "sensor_msgs/msg/Imu", rmw_get_serialization_format(), ""});
//    writer_->create_topic({"kitti/oxts/gps_fix", "sensor_msgs/msg/NavSatFix", rmw_get_serialization_format(), ""});
//    writer_->create_topic({"kitti/oxts/vel", "geometry_msgs/msg/TwistStamped", rmw_get_serialization_format(), ""});
//  } else if (dir == "velodyne_points") {
//    writer_->create_topic({"kitti/velo", "sensor_msgs/msg/PointCloud2", rmw_get_serialization_format(), ""});
    }
  }

  timer_ = create_wall_timer(100ms, std::bind(&Kitti2BagNode::on_timer_callback, this));
};

void Kitti2BagNode::on_timer_callback()
{
  for (size_t i = 0; i < dirs_.size(); ++i) {
    const std::string & dir = dirs_[i];
    fs::path file_path = kitti_path_ / dir / "data" / filenames_[i][index_];
    rclcpp::Time timestamp = timestamps_[i][index_];

    if (dir == "image_00") {
      cv::Mat image = cv::imread(file_path, cv::IMREAD_UNCHANGED);
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();
      msg->header.frame_id = "cam0";
      msg->header.stamp = timestamp;
      writer_->write(*msg, "kitti/camera_gray_left", timestamp);
    } else if (dir == "image_01") {
      cv::Mat image = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();
      msg->header.frame_id = "cam1";
      msg->header.stamp = timestamp;
      writer_->write(*msg, "kitti/camera_gray_right", timestamp);
    } else if (dir == "image_02") {
      cv::Mat image = cv::imread(file_path, cv::IMREAD_UNCHANGED);
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
      msg->header.frame_id = "cam2";
      msg->header.stamp = timestamp;
      writer_->write(*msg, "kitti/camera_color_left", timestamp);
    } else if (dir == "image_03") {
      cv::Mat image = cv::imread(file_path, cv::IMREAD_COLOR);
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
      msg->header.frame_id = "cam3";
      msg->header.stamp = timestamp;
      writer_->write(*msg, "kitti/camera_color_right", timestamp);
    } else if (dir == "oxts") {
    } else if (dir == "velodyne_points") {
    }
  }

  RCLCPP_INFO(this->get_logger(), "Publishing index = %ld", index_);
  ++index_;
  if (index_ == max_index_) {
    timer_->cancel();
    rclcpp::shutdown();
  }
}

void Kitti2BagNode::get_filenames()
{
  for (auto & dir : dirs_) {
    std::vector<std::string> filenames_vec{};
    try {
      fs::path directory_path = kitti_path_ / dir / "data";
      if (fs::is_directory(directory_path)) {
        for (const auto & entry : fs::directory_iterator(directory_path)) {
          if (entry.is_regular_file()) {
            filenames_vec.push_back(entry.path().filename().string());
          }
        }
        // sort the filename
        std::sort(filenames_vec.begin(), filenames_vec.end(),
          [](const auto & lhs, const auto & rhs) {
            return lhs < rhs;
          });
      } else {
        RCLCPP_ERROR(get_logger(), "Error: The specified path is not a directory.");
        rclcpp::shutdown();
      }
    } catch (const fs::filesystem_error & e) {
      RCLCPP_ERROR(get_logger(), "Filesystem error: %s", e.what());
      rclcpp::shutdown();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
      rclcpp::shutdown();
    }
    filenames_.push_back(filenames_vec);
  }

  // make sure all folders have the same number of files
  for (size_t i = 0; i < filenames_.size()-1; ++i) {
    if (filenames_[i].size() != filenames_[i+1].size()) {
      RCLCPP_ERROR(get_logger(),
        "File size in %s (%ld) and %s (%ld) don't match.",
        dirs_[i].c_str(), filenames_[i].size(), dirs_[i+1].c_str(), filenames_[i+1].size());
      rclcpp::shutdown();
    }
  }
  max_index_ = filenames_[0].size();
}

void Kitti2BagNode::get_all_timestamps()
{
  for (auto & dir : dirs_) {
    std::vector<rclcpp::Time> timestamps_vec{};

    // open the file
    std::ifstream input_file(kitti_path_ / dir / "timestamps.txt");
    if (!input_file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Unable to open file timestamps.txt");
      rclcpp::shutdown();
    }

    std::string line;
    while (std::getline(input_file, line)) {
      // Convert timestamp string to std::chrono::time_point
      std::tm tm = {};
      std::istringstream iss(line);
      iss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S.");
      if (iss.fail()) {
        RCLCPP_ERROR(get_logger(), "Failed to parse timestamp.");
        rclcpp::shutdown();
      }

      // Extract microseconds and convert to duration
      std::chrono::nanoseconds::rep nanoSecondsCount;
      iss >> nanoSecondsCount;
      if (iss.fail()) {
        RCLCPP_ERROR(get_logger(), "Failed to parse microseconds.");
        rclcpp::shutdown();
      }
      auto nanoseconds = std::chrono::nanoseconds(nanoSecondsCount).count();

      // Convert std::tm to std::chrono::time_point
      auto timePoint = std::chrono::system_clock::from_time_t(std::mktime(&tm));

      // Convert time_point to epoch time (seconds since 1970-01-01)
      auto epochTime = std::chrono::duration_cast<std::chrono::seconds>(timePoint.time_since_epoch()).count();

      timestamps_vec.push_back(rclcpp::Time(epochTime, nanoseconds));
    }
    timestamps_.push_back(timestamps_vec);
  }
}
