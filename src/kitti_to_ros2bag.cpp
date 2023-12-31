#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "kitti_to_ros2bag/kitti_to_ros2bag.hpp"


using namespace std::chrono_literals;
namespace fs = std::filesystem;

Kitti2BagNode::Kitti2BagNode()
: Node("kitti2bag_node"), index_{0}
{
  declare_parameter("kitti_path", rclcpp::PARAMETER_STRING);
  kitti_path_ = get_parameter("kitti_path").as_string();

  dirs_ = {"image_00/", "image_01/", "image_02/", "image_03/", "oxts/", "velodyne_points/"};

  get_filenames();
  max_index_ = filenames_.size();

  get_all_timestamps();

  timer_ = create_wall_timer(100ms, std::bind(&Kitti2BagNode::on_timer_callback, this));
};

void Kitti2BagNode::on_timer_callback()
{
  // image_00
  // image_01
  // image_02
  // image_03
  // oxts
  // velodyne_points

//// OXTS to IMU and GPS
//std::string oxts_filename = oxts_path_ + oxts_filenams_[index_];


  RCLCPP_INFO(this->get_logger(), "Publishing index = %ld", index_);
  ++index_;
  if (index_ == max_index_) {
    timer_->cancel();
    rclcpp::shutdown();
  }
}

void Kitti2BagNode::get_filenames()
{
  try {
    fs::path directory_path = kitti_path_ + dirs_[0] + "data";
    if (fs::is_directory(directory_path)) {
      for (const auto & entry : fs::directory_iterator(directory_path)) {
        if (entry.is_regular_file()) {
          filenames_.push_back(entry.path().filename().stem());
        }
      }
      // sort the filename
      std::sort(filenames_.begin(), filenames_.end(),
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
}

void Kitti2BagNode::get_all_timestamps()
{
  const std::string timestamps_file = "timestamps.txt";
  for (auto & dir : dirs_) {
    std::vector<rclcpp::Time> timestamp_vec{};

    // open the file
    std::ifstream input_file(kitti_path_ + dir + timestamps_file);
    if (!input_file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Unable to open file %s", timestamps_file.c_str());
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

      timestamp_vec.push_back(rclcpp::Time(epochTime, nanoseconds));
    }
    timestamp_.push_back(timestamp_vec);
  }
}
