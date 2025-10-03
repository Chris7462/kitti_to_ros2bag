#pragma once

// C++
#include <vector>
#include <string>
#include <filesystem>

// ROS 2
#include <rclcpp/rclcpp.hpp>

namespace fs = std::filesystem;

class DataLoader
{
public:
  DataLoader(const fs::path & kitti_path, const std::string & data_folder,
    const std::vector<std::string> & dirs, const rclcpp::Logger & logger);

  void load_filenames();
  void load_timestamps();

  size_t get_max_index() const { return max_index_; }
  const std::vector<std::vector<std::string>> & get_filenames() const { return filenames_; }
  const std::vector<std::vector<rclcpp::Time>> & get_timestamps() const { return timestamps_; }

private:
  fs::path kitti_path_;
  std::string data_folder_;
  std::vector<std::string> dirs_;
  rclcpp::Logger logger_;
  size_t max_index_;
  std::vector<std::vector<std::string>> filenames_;
  std::vector<std::vector<rclcpp::Time>> timestamps_;
};
