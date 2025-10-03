// local
#include "kitti_to_ros2bag/kitti_to_ros2bag.hpp"

using namespace std::chrono_literals;

Kitti2BagNode::Kitti2BagNode()
: Node("kitti2bag_node"), index_{0}
{
  kitti_path_ = declare_parameter("kitti_path", fs::path());
  data_folder_ = declare_parameter("data_folder", std::string());
  calib_folder_ = declare_parameter("calib_folder", std::string());
  dirs_ = declare_parameter("dirs", std::vector<std::string>());
  std::string output_bag_name = data_folder_ + "_bag";

  // Create helper objects
  data_loader_ = std::make_unique<DataLoader>(kitti_path_, data_folder_, dirs_, get_logger());
  message_converter_ = std::make_unique<MessageConverter>(get_logger());
  bag_writer_ = std::make_unique<BagWriter>(output_bag_name);

  // Load data
  data_loader_->load_filenames();
  data_loader_->load_timestamps();
  max_index_ = data_loader_->get_max_index();

  filenames_ = data_loader_->get_filenames();
  timestamps_ = data_loader_->get_timestamps();

  // Create topics
  bag_writer_->create_topics();

  timer_ = create_wall_timer(100ms, std::bind(&Kitti2BagNode::on_timer_callback, this));
}

void Kitti2BagNode::on_timer_callback()
{
  fs::path calib_file = kitti_path_ / calib_folder_ / "calib_cam_to_cam.txt";
  auto calib_flag = fs::exists(calib_file);

  for (size_t i = 0; i < dirs_.size(); ++i) {
    const std::string & dir = dirs_[i];
    fs::path filename = kitti_path_ / data_folder_ / dir / "data" / filenames_[i][index_];
    rclcpp::Time timestamp = timestamps_[i][index_];

    if (dir == "image_00") {
      auto msg = message_converter_->convert_image_to_msg(filename, timestamp, "mono8", "cam0_link");
      bag_writer_->write_message(msg, "/kitti/camera/gray/left/image_raw", timestamp);
      if (calib_flag) {
        auto msg = message_converter_->convert_calib_to_msg(calib_file, timestamp, "0", "cam0_link");
        rclcpp::Time tmp = rclcpp::Time(timestamp.nanoseconds() + 10);
        bag_writer_->write_message(msg, "/kitti/camera/gray/left/camera_info", tmp);
      }
    } else if (dir == "image_01") {
      auto msg = message_converter_->convert_image_to_msg(filename, timestamp, "mono8", "cam1_link");
      bag_writer_->write_message(msg, "/kitti/camera/gray/right/image_raw", timestamp);
      if (calib_flag) {
        auto msg = message_converter_->convert_calib_to_msg(calib_file, timestamp, "1", "cam1_link");
        rclcpp::Time tmp = rclcpp::Time(timestamp.nanoseconds() + 10);
        bag_writer_->write_message(msg, "/kitti/camera/gray/right/camera_info", tmp);
      }
    } else if (dir == "image_02") {
      auto msg = message_converter_->convert_image_to_msg(filename, timestamp, "bgr8", "cam2_link");
      bag_writer_->write_message(msg, "/kitti/camera/color/left/image_raw", timestamp);
      if (calib_flag) {
        auto msg = message_converter_->convert_calib_to_msg(calib_file, timestamp, "2", "cam2_link");
        rclcpp::Time tmp = rclcpp::Time(timestamp.nanoseconds() + 10);
        bag_writer_->write_message(msg, "/kitti/camera/color/left/camera_info", tmp);
      }
    } else if (dir == "image_03") {
      auto msg = message_converter_->convert_image_to_msg(filename, timestamp, "bgr8", "cam3_link");
      bag_writer_->write_message(msg, "/kitti/camera/color/right/image_raw", timestamp);
      if (calib_flag) {
        auto msg = message_converter_->convert_calib_to_msg(calib_file, timestamp, "3", "cam3_link");
        rclcpp::Time tmp = rclcpp::Time(timestamp.nanoseconds() + 10);
        bag_writer_->write_message(msg, "/kitti/camera/color/right/camera_info", tmp);
      }
    } else if (dir == "oxts") {
      // parse the oxts data
      std::vector<std::string> oxts_parsed_array = message_converter_->parse_file_data(filename);

      // write the GPS data to bag
      auto gps_msg = message_converter_->convert_oxts_to_gps_msg(oxts_parsed_array, timestamp);
      bag_writer_->write_message(gps_msg, "/kitti/oxts/gps/fix", timestamp);

      // write the velocity data to bag
      auto vel_msg = message_converter_->convert_oxts_to_vel_msg(oxts_parsed_array, timestamp);
      rclcpp::Time tmp1 = rclcpp::Time(timestamp.nanoseconds() + 10);
      bag_writer_->write_message(vel_msg, "/kitti/oxts/gps/vel", tmp1);

      // write the IMU data to bag
      auto imu_msg = message_converter_->convert_oxts_to_imu_msg(oxts_parsed_array, timestamp);
      rclcpp::Time tmp2 = rclcpp::Time(timestamp.nanoseconds() + 20);
      bag_writer_->write_message(imu_msg, "/kitti/oxts/imu", tmp2);
    } else if (dir == "velodyne_points") {
      auto msg = message_converter_->convert_velo_to_msg(filename, timestamp);
      bag_writer_->write_message(msg, "/kitti/velo", timestamp);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Writing index = %ld", index_);
  ++index_;
  if (index_ == max_index_) {
    timer_->cancel();
    rclcpp::shutdown();
  }
}
