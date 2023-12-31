#include <chrono>
#include <filesystem>
#include <algorithm>

#include "kitti_to_ros2bag/kitti_to_ros2bag.hpp"


using namespace std::chrono_literals;
namespace fs = std::filesystem;

Kitti2BagNode::Kitti2BagNode()
: Node("kitti2bag_node"), index_{0}
{
  declare_parameter("oxts_path", rclcpp::PARAMETER_STRING);
  declare_parameter("point_cloud_path", rclcpp::PARAMETER_STRING);
  declare_parameter("left_gray_image_path", rclcpp::PARAMETER_STRING);
  declare_parameter("right_gray_image_path", rclcpp::PARAMETER_STRING);
  declare_parameter("left_color_image_path", rclcpp::PARAMETER_STRING);
  declare_parameter("right_color_image_path", rclcpp::PARAMETER_STRING);

  oxts_path_ = get_parameter("oxts_path").as_string();
  point_cloud_path_ = get_parameter("point_cloud_path").as_string();
  left_gray_image_path_ = get_parameter("left_gray_image_path").as_string();
  right_gray_image_path_ = get_parameter("right_gray_image_path").as_string();
  left_color_image_path_ = get_parameter("left_color_image_path").as_string();
  right_color_image_path_ = get_parameter("right_color_image_path").as_string();

  get_all_filenames();
  max_index_ = oxts_filenams_.size();

  timer_ = create_wall_timer(100ms, std::bind(&Kitti2BagNode::on_timer_callback, this));
};

void Kitti2BagNode::on_timer_callback()
{
  // OXTS to IMU and GPS
  std::string oxts_filename = oxts_path_ + oxts_filenams_[index_];


  RCLCPP_INFO(this->get_logger(), "Publishing index = %ld", index_);
  ++index_;
  if (index_ == max_index_) {
    timer_->cancel();
    rclcpp::shutdown();
  }
}

void Kitti2BagNode::get_all_filenames()
{
  for (Type currentType = oxts; currentType < maxTypes; currentType = static_cast<Type>(currentType + 1)) {
    fs::path directory_path;
    std::vector<std::string>* filenames;

    switch (currentType) {
      case oxts:
        directory_path = oxts_path_;
        filenames = &oxts_filenams_;
        break;
      case point_cloud:
        directory_path = point_cloud_path_;
        filenames = &point_cloud_filenams_;
        break;
      case left_gray_image:
        directory_path = left_gray_image_path_;
        filenames = &left_gray_image_filenams_;
        break;
      case right_gray_image:
        directory_path = right_gray_image_path_;
        filenames = &right_gray_image_filenams_;
        break;
      case left_color_image:
        directory_path = left_color_image_path_;
        filenames = &left_color_image_filenams_;
        break;
      case right_color_image:
        directory_path = right_color_image_path_;
        filenames = &right_color_image_filenams_;
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unexpected enum");
        break;
    }

    try {
      if (fs::is_directory(directory_path)) {
        for (const auto & entry : fs::directory_iterator(directory_path)) {
          if (entry.is_regular_file()) {
            filenames->push_back(entry.path().filename().string());
          }
        }
        // sort the filename
        std::sort(filenames->begin(), filenames->end(),
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
}
