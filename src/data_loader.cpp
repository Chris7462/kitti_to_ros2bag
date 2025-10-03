// C++
#include <algorithm>
#include <fstream>
#include <sstream>
#include <ctime>

// local
#include "kitti_to_ros2bag/data_loader.hpp"


DataLoader::DataLoader(const fs::path & kitti_path, const std::string & data_folder,
  const std::vector<std::string> & dirs, const rclcpp::Logger & logger)
: kitti_path_(kitti_path), data_folder_(data_folder), dirs_(dirs), logger_(logger), max_index_(0)
{
}

void DataLoader::load_filenames()
{
  for (auto & dir : dirs_) {
    std::vector<std::string> filenames_vec{};
    try {
      fs::path directory_path = kitti_path_ / data_folder_ / dir / "data";
      if (fs::is_directory(directory_path)) {
        for (const auto & entry : fs::directory_iterator(directory_path)) {
          if (entry.is_regular_file()) {
            filenames_vec.push_back(entry.path().filename().string());
          }
        }
        // sort the filename
        std::sort(
          filenames_vec.begin(), filenames_vec.end(),
          [](const auto & lhs, const auto & rhs) {
            return lhs < rhs;
          });
      } else {
        RCLCPP_ERROR(logger_, "Error: The specified path is not a directory.");
        rclcpp::shutdown();
      }
    } catch (const fs::filesystem_error & e) {
      RCLCPP_ERROR(logger_, "Filesystem error: %s", e.what());
      rclcpp::shutdown();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Error: %s", e.what());
      rclcpp::shutdown();
    }
    filenames_.push_back(filenames_vec);
  }

  // make sure all folders have the same number of files
  for (size_t i = 0; i < filenames_.size() - 1; ++i) {
    if (filenames_[i].size() != filenames_[i + 1].size()) {
      RCLCPP_ERROR(
        logger_,
        "File size in %s (%ld) and %s (%ld) don't match.",
        dirs_[i].c_str(), filenames_[i].size(), dirs_[i + 1].c_str(), filenames_[i + 1].size());
      rclcpp::shutdown();
    }
  }
  max_index_ = filenames_[0].size();
}

void DataLoader::load_timestamps()
{
  for (auto & dir : dirs_) {
    std::vector<rclcpp::Time> timestamps_vec{};

    // open the file
    std::ifstream input_file(kitti_path_ / data_folder_ / dir / "timestamps.txt");
    if (!input_file.is_open()) {
      RCLCPP_ERROR(logger_, "Unable to open file timestamps.txt");
      rclcpp::shutdown();
    }

    std::string line;
    while (std::getline(input_file, line)) {
      // Convert timestamp string to std::chrono::time_point
      std::tm tm = {};
      std::istringstream iss(line);
      iss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S.");
      if (iss.fail()) {
        RCLCPP_ERROR(logger_, "Failed to parse timestamp.");
        rclcpp::shutdown();
      }

      // Extract nanoseconds and convert to duration
      std::chrono::nanoseconds::rep nanoSecondsCount;
      iss >> nanoSecondsCount;
      if (iss.fail()) {
        RCLCPP_ERROR(logger_, "Failed to parse nanoseconds.");
        rclcpp::shutdown();
      }
      auto nanoseconds = std::chrono::nanoseconds(nanoSecondsCount).count();

      // Convert std::tm to std::chrono::time_point
      auto timePoint = std::chrono::system_clock::from_time_t(std::mktime(&tm));

      // Convert time_point to epoch time (seconds since 1970-01-01)
      auto epochTime =
        std::chrono::duration_cast<std::chrono::seconds>(timePoint.time_since_epoch()).count();

      timestamps_vec.push_back(rclcpp::Time(epochTime, nanoseconds));
    }
    timestamps_.push_back(timestamps_vec);
  }
}
