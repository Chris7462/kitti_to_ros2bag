// C++ header
#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>

// OpenCV header
#include <opencv2/imgcodecs.hpp>

// PCL header
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ROS header
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

// local header
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

  get_filenames();
  get_all_timestamps();

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(output_bag_name);

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
      auto msg = convert_image_to_msg(filename, timestamp, "mono8", "cam0_link");
      writer_->write(msg, "kitti/camera/gray/left/image_raw", timestamp);
      if (calib_flag) {
        auto msg = convert_calib_to_msg(calib_file, timestamp, "0", "cam0_link");
        // Have to add some offset to the timestamp, otherwise it will overwrite the previous message
        rclcpp::Time tmp = rclcpp::Time(timestamp.nanoseconds() + 10);
        writer_->write(msg, "kitti/camera/gray/left/camera_info", tmp);
      }
    } else if (dir == "image_01") {
      auto msg = convert_image_to_msg(filename, timestamp, "mono8", "cam1_link");
      writer_->write(msg, "kitti/camera/gray/right/image_raw", timestamp);
      if (calib_flag) {
        auto msg = convert_calib_to_msg(calib_file, timestamp, "1", "cam1_link");
        rclcpp::Time tmp = rclcpp::Time(timestamp.nanoseconds() + 10);
        writer_->write(msg, "kitti/camera/gray/right/camera_info", tmp);
      }
    } else if (dir == "image_02") {
      auto msg = convert_image_to_msg(filename, timestamp, "bgr8", "cam2_link");
      writer_->write(msg, "kitti/camera/color/left/image_raw", timestamp);
      if (calib_flag) {
        auto msg = convert_calib_to_msg(calib_file, timestamp, "2", "cam2_link");
        rclcpp::Time tmp = rclcpp::Time(timestamp.nanoseconds() + 10);
        writer_->write(msg, "kitti/camera/color/left/camera_info", tmp);
      }
    } else if (dir == "image_03") {
      auto msg = convert_image_to_msg(filename, timestamp, "bgr8", "cam3_link");
      writer_->write(msg, "kitti/camera/color/right/image_raw", timestamp);
      if (calib_flag) {
        auto msg = convert_calib_to_msg(calib_file, timestamp, "3", "cam3_link");
        rclcpp::Time tmp = rclcpp::Time(timestamp.nanoseconds() + 10);
        writer_->write(msg, "kitti/camera/color/right/camera_info", tmp);
      }
    } else if (dir == "oxts") {
      // parse the oxts data
      std::vector<std::string> oxts_parsed_array = parse_file_data(filename, " ");

      // write the GPS data to bag
      auto gps_msg = convert_oxts_to_gps_msg(oxts_parsed_array, timestamp);
      writer_->write(gps_msg, "kitti/oxts/gps/fix", timestamp);

      // write the velocity data to bag
      auto vel_msg = convert_oxts_to_vel_msg(oxts_parsed_array, timestamp);
      rclcpp::Time tmp1 = rclcpp::Time(timestamp.nanoseconds() + 10);
      writer_->write(vel_msg, "kitti/oxts/gps/vel", tmp1);

      // write the IMU data to bag
      auto img_msg = convert_oxts_to_imu_msg(oxts_parsed_array, timestamp);
      rclcpp::Time tmp2 = rclcpp::Time(timestamp.nanoseconds() + 20);
      writer_->write(img_msg, "kitti/oxts/imu", tmp2);
    } else if (dir == "velodyne_points") {
      auto msg = convert_velo_to_msg(filename, timestamp);
      writer_->write(msg, "kitti/velo", timestamp);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Writing index = %ld", index_);
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
  for (size_t i = 0; i < filenames_.size() - 1; ++i) {
    if (filenames_[i].size() != filenames_[i + 1].size()) {
      RCLCPP_ERROR(
        get_logger(),
        "File size in %s (%ld) and %s (%ld) don't match.",
        dirs_[i].c_str(), filenames_[i].size(), dirs_[i + 1].c_str(), filenames_[i + 1].size());
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
    std::ifstream input_file(kitti_path_ / data_folder_ / dir / "timestamps.txt");
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
      auto epochTime =
        std::chrono::duration_cast<std::chrono::seconds>(timePoint.time_since_epoch()).count();

      timestamps_vec.push_back(rclcpp::Time(epochTime, nanoseconds));
    }
    timestamps_.push_back(timestamps_vec);
  }
}

sensor_msgs::msg::Image Kitti2BagNode::convert_image_to_msg(
  const fs::path & file_path, const rclcpp::Time & timestamp,
  const std::string & encoding, const std::string & frame_id)
{
  cv::Mat image = cv::imread(file_path, cv::IMREAD_UNCHANGED);
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image).toImageMsg();
  msg->header.frame_id = frame_id;
  msg->header.stamp = timestamp;

  return *msg;
}

std::vector<std::string> Kitti2BagNode::parse_file_data(
  const fs::path & file_path, std::string delimiter)
{
  std::ifstream input_file(file_path);

  if (!input_file.good()) {
    RCLCPP_ERROR(this->get_logger(), "Could not read OXTS data. Check your file path!");
    rclcpp::shutdown();
  }

  std::string file_content_string;
  if (input_file) {
    std::ostringstream ss;
    ss << input_file.rdbuf(); // reading data
    file_content_string = ss.str();
  }

  //https://www.codegrepper.com/code-examples/whatever/c%2B%2B+how+to+tokenize+a+string
  std::vector<std::string> tokens;
  size_t first = 0;
  while (first < file_content_string.size()) {
    size_t second = file_content_string.find_first_of(delimiter, first);
    //first has index of start of token
    //second has index of end of token + 1;
    if (second == std::string::npos) {
      second = file_content_string.size();
    }
    std::string token = file_content_string.substr(first, second - first);
    tokens.push_back(token);
    first = second + 1;
  }

  return tokens;
}

sensor_msgs::msg::NavSatFix Kitti2BagNode::convert_oxts_to_gps_msg(
  const std::vector<std::string> & oxts_tokenized_array,
  const rclcpp::Time & timestamp)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = "gps_link";

  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  msg.latitude = std::atof(oxts_tokenized_array[0].c_str());
  msg.longitude = std::atof(oxts_tokenized_array[1].c_str());
  msg.altitude = std::atof(oxts_tokenized_array[2].c_str());

  msg.position_covariance[0] = std::atof(oxts_tokenized_array[23].c_str());
  msg.position_covariance[1] = 0.0f;
  msg.position_covariance[2] = 0.0f;
  msg.position_covariance[3] = 0.0f;
  msg.position_covariance[4] = std::atof(oxts_tokenized_array[23].c_str());
  msg.position_covariance[5] = 0.0f;
  msg.position_covariance[6] = 0.0f;
  msg.position_covariance[7] = 0.0f;
  msg.position_covariance[8] = std::atof(oxts_tokenized_array[23].c_str());

  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

  return msg;
}

geometry_msgs::msg::TwistStamped Kitti2BagNode::convert_oxts_to_vel_msg(
  const std::vector<std::string> & oxts_tokenized_array,
  const rclcpp::Time & timestamp)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = "gps_link";

  msg.twist.linear.x = std::atof(oxts_tokenized_array[8].c_str());
  msg.twist.linear.y = std::atof(oxts_tokenized_array[9].c_str());
  msg.twist.linear.z = std::atof(oxts_tokenized_array[10].c_str());

  msg.twist.angular.x = std::atof(oxts_tokenized_array[20].c_str());
  msg.twist.angular.y = std::atof(oxts_tokenized_array[21].c_str());
  msg.twist.angular.z = std::atof(oxts_tokenized_array[22].c_str());

  return msg;
}

sensor_msgs::msg::Imu Kitti2BagNode::convert_oxts_to_imu_msg(
  const std::vector<std::string> & oxts_tokenized_array,
  const rclcpp::Time & timestamp)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = "imu_link";

  // - roll:  roll angle (rad),  0 = level, positive = left side up,      range: -pi   .. +pi
  // - pitch: pitch angle (rad), 0 = level, positive = front down,        range: -pi/2 .. +pi/2
  // - yaw:   heading (rad),     0 = east,  positive = counter clockwise, range: -pi   .. +pi
  tf2::Quaternion orientation;
  orientation.setRPY(
    std::atof(oxts_tokenized_array[3].c_str()),
    std::atof(oxts_tokenized_array[4].c_str()),
    std::atof(oxts_tokenized_array[5].c_str()));
  msg.orientation = tf2::toMsg(orientation);

  // - wf:    angular rate around forward axis (rad/s)
  // - wl:    angular rate around leftward axis (rad/s)
  // - wu:    angular rate around upward axis (rad/s)
  msg.angular_velocity.x = std::atof(oxts_tokenized_array[20].c_str());
  msg.angular_velocity.y = std::atof(oxts_tokenized_array[21].c_str());
  msg.angular_velocity.z = std::atof(oxts_tokenized_array[22].c_str());

  // - af:    forward acceleration (m/s^2)
  // - al:    leftward acceleration (m/s^2)
  // - au:    upward acceleration (m/s^2)
  msg.linear_acceleration.x = std::atof(oxts_tokenized_array[14].c_str());
  msg.linear_acceleration.y = std::atof(oxts_tokenized_array[15].c_str());
  msg.linear_acceleration.z = std::atof(oxts_tokenized_array[16].c_str());

  return msg;
}

sensor_msgs::msg::PointCloud2 Kitti2BagNode::convert_velo_to_msg(
  const fs::path & file_path, const rclcpp::Time & timestamp)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  std::fstream input_file(file_path, std::ios::in | std::ios::binary);
  if (!input_file.good()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Could not read Velodyne's point cloud. Check your file path!");
    rclcpp::shutdown();
  }

  input_file.seekg(0, std::ios::beg);
  while (input_file.good() && !input_file.eof()) {
    pcl::PointXYZI point;
    input_file.read((char *) &point.x, sizeof(float));
    input_file.read((char *) &point.y, sizeof(float));
    input_file.read((char *) &point.z, sizeof(float));
    input_file.read((char *) &point.intensity, sizeof(float));
    cloud.push_back(point);
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "velo_link";
  msg.header.stamp = timestamp;

  return msg;
}

sensor_msgs::msg::CameraInfo Kitti2BagNode::convert_calib_to_msg(
  const std::string & calib_file, const rclcpp::Time & timestamp,
  const std::string & id, const std::string & frame_id)
{
  // open the file
  std::ifstream input_file(calib_file);
  if (!input_file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Unable to open file calib_cam_to_cam.txt");
    rclcpp::shutdown();
  }

  CalibrationData calib;

  std::string line;
  std::string tmp_str;
  while (std::getline(input_file, line)) {
    std::size_t pos = line.find(':');
    if (pos != std::string::npos) {
      std::string key = line.substr(0, pos);
      std::string value = line.substr(pos + 1);

      // Trim leading and trailing whitespaces
      key.erase(0, key.find_first_not_of(" \t\r\n"));
      key.erase(key.find_last_not_of(" \t\r\n") + 1);
      value.erase(0, value.find_first_not_of(" \t\r\n"));
      value.erase(value.find_last_not_of(" \t\r\n") + 1);

      // get the value of the calibration and put them into stringstream
      std::stringstream ss(value);

      int j = 0;
      if (key == "K_0" + id) {
        while (getline(ss, tmp_str, ' ')) {
          calib.K[j] = std::stod(tmp_str);
          ++j;
        }
      } else if (key == "D_0" + id) {
        while (getline(ss, tmp_str, ' ')) {
          calib.D.push_back(std::stod(tmp_str));
        }
      } else if (key == "S_rect_0" + id) {
        while (getline(ss, tmp_str, ' ')) {
          calib.S_rect[j] = std::stod(tmp_str);
          ++j;
        }
      } else if (key == "R_rect_0" + id) {
        while (getline(ss, tmp_str, ' ')) {
          calib.R_rect[j] = std::stod(tmp_str);
          ++j;
        }
      } else if (key == "P_rect_0" + id) {
        while (getline(ss, tmp_str, ' ')) {
          calib.P_rect[j] = std::stod(tmp_str);
          ++j;
        }
      }
    }
  }
  // Close the file
  input_file.close();

  // Ready to output to message
  sensor_msgs::msg::CameraInfo calib_msg;
  calib_msg.header.frame_id = frame_id;
  calib_msg.header.stamp = timestamp;
  calib_msg.width = calib.S_rect[0];
  calib_msg.height = calib.S_rect[1];
  calib_msg.distortion_model = "plumb_bob";
  calib_msg.k = calib.K;
  calib_msg.r = calib.R_rect;
  calib_msg.d = calib.D;
  calib_msg.p = calib.P_rect;

  return calib_msg;
}
