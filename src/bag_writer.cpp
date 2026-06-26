// ROS 2
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

// local
#include "kitti_to_ros2bag/bag_writer.hpp"


BagWriter::BagWriter(const std::string & output_bag_name)
{
  writer_ = std::make_unique<rosbag2_cpp::Writer>();

  // Configure storage options for MCAP format
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = output_bag_name;
  storage_options.storage_id = "mcap";

  // Configure converter options
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  writer_->open(storage_options, converter_options);
}

void BagWriter::create_topics()
{
  // SensorDataQoS: best_effort, volatile, keep_last(5)
  // Used for high-bandwidth streams where dropping old frames is acceptable.
  const std::vector<rclcpp::QoS> sensor_qos = {rclcpp::SensorDataQoS().keep_last(5)};

  // Reliable QoS, keep_last(10)
  // Used for low-rate metadata (camera_info, GPS) where every message matters.
  const std::vector<rclcpp::QoS> reliable_qos = {
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile()};

  // Camera image topics
  create_topic("/kitti/camera/gray/left/image_raw", "sensor_msgs/msg/Image", sensor_qos);
  create_topic("/kitti/camera/gray/left/camera_info", "sensor_msgs/msg/CameraInfo", reliable_qos);
  create_topic("/kitti/camera/gray/right/image_raw", "sensor_msgs/msg/Image", sensor_qos);
  create_topic("/kitti/camera/gray/right/camera_info", "sensor_msgs/msg/CameraInfo", reliable_qos);
  create_topic("/kitti/camera/color/left/image_raw", "sensor_msgs/msg/Image", sensor_qos);
  create_topic("/kitti/camera/color/left/camera_info", "sensor_msgs/msg/CameraInfo", reliable_qos);
  create_topic("/kitti/camera/color/right/image_raw", "sensor_msgs/msg/Image", sensor_qos);
  create_topic("/kitti/camera/color/right/camera_info", "sensor_msgs/msg/CameraInfo", reliable_qos);

  // OXTS topics
  create_topic("/kitti/oxts/gps/fix", "sensor_msgs/msg/NavSatFix", reliable_qos);
  create_topic("/kitti/oxts/gps/vel", "geometry_msgs/msg/TwistStamped", reliable_qos);
  create_topic("/kitti/oxts/imu", "sensor_msgs/msg/Imu", reliable_qos);

  // Velodyne topic
  create_topic("/kitti/velo", "sensor_msgs/msg/PointCloud2", sensor_qos);
}

void BagWriter::create_topic(
  const std::string & topic_name,
  const std::string & topic_type,
  const std::vector<rclcpp::QoS> & qos_profiles)
{
  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = topic_name;
  topic_metadata.type = topic_type;
  topic_metadata.serialization_format = "cdr";
  topic_metadata.offered_qos_profiles = qos_profiles;
  // type_description_hash is a required field added in post-Jazzy rosbag2.
  // Empty string is valid when not recording live type introspection data.
  topic_metadata.type_description_hash = "";
  writer_->create_topic(topic_metadata);
}
