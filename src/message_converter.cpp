// C++ Standard Library
#include <fstream>
#include <sstream>

// OpenCV
#include <opencv2/imgcodecs.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ROS 2
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <pcl_conversions/pcl_conversions.h>

// Local
#include "kitti_to_ros2bag/message_converter.hpp"


MessageConverter::MessageConverter(const rclcpp::Logger & logger)
: logger_(logger)
{
}

sensor_msgs::msg::Image MessageConverter::convert_image_to_msg(
  const fs::path & file_path, const rclcpp::Time & timestamp,
  const std::string & encoding, const std::string & frame_id)
{
  cv::Mat image = cv::imread(file_path, cv::IMREAD_UNCHANGED);
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image).toImageMsg();
  msg->header.frame_id = frame_id;
  msg->header.stamp = timestamp;

  return *msg;
}

std::vector<std::string> MessageConverter::parse_file_data(const fs::path & file_path)
{
  std::ifstream input_file(file_path);

  if (!input_file.good()) {
    RCLCPP_ERROR(logger_, "Could not read OXTS data. Check your file path!");
    rclcpp::shutdown();
  }

  std::string file_content_string;
  if (input_file) {
    std::ostringstream ss;
    ss << input_file.rdbuf();
    file_content_string = ss.str();
  }

  //https://www.codegrepper.com/code-examples/whatever/c%2B%2B+how+to+tokenize+a+string
  std::vector<std::string> tokens;
  size_t first = 0;
  while (first < file_content_string.size()) {
    size_t second = file_content_string.find_first_of(' ', first);
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

sensor_msgs::msg::NavSatFix MessageConverter::convert_oxts_to_gps_msg(
  const std::vector<std::string> & oxts_tokenized_array,
  const rclcpp::Time & timestamp)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = "oxts_link";

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

geometry_msgs::msg::TwistStamped MessageConverter::convert_oxts_to_vel_msg(
  const std::vector<std::string> & oxts_tokenized_array,
  const rclcpp::Time & timestamp)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = "oxts_link";

  msg.twist.linear.x = std::atof(oxts_tokenized_array[8].c_str());
  msg.twist.linear.y = std::atof(oxts_tokenized_array[9].c_str());
  msg.twist.linear.z = std::atof(oxts_tokenized_array[10].c_str());

  msg.twist.angular.x = std::atof(oxts_tokenized_array[20].c_str());
  msg.twist.angular.y = std::atof(oxts_tokenized_array[21].c_str());
  msg.twist.angular.z = std::atof(oxts_tokenized_array[22].c_str());

  return msg;
}

sensor_msgs::msg::Imu MessageConverter::convert_oxts_to_imu_msg(
  const std::vector<std::string> & oxts_tokenized_array,
  const rclcpp::Time & timestamp)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = "oxts_link";

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

sensor_msgs::msg::PointCloud2 MessageConverter::convert_velo_to_msg(
  const fs::path & file_path, const rclcpp::Time & timestamp)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  std::fstream input_file(file_path, std::ios::in | std::ios::binary);
  if (!input_file.good()) {
    RCLCPP_ERROR(
      logger_,
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

sensor_msgs::msg::CameraInfo MessageConverter::convert_calib_to_msg(
  const std::string & calib_file, const rclcpp::Time & timestamp,
  const std::string & id, const std::string & frame_id)
{
  // open the file
  std::ifstream input_file(calib_file);
  if (!input_file.is_open()) {
    RCLCPP_ERROR(logger_, "Unable to open file calib_cam_to_cam.txt");
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
