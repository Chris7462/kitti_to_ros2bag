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
  // Camera image topics
  create_topic("/kitti/camera/gray/left/image_raw", "sensor_msgs/msg/Image");
  create_topic("/kitti/camera/gray/left/camera_info", "sensor_msgs/msg/CameraInfo");
  create_topic("/kitti/camera/gray/right/image_raw", "sensor_msgs/msg/Image");
  create_topic("/kitti/camera/gray/right/camera_info", "sensor_msgs/msg/CameraInfo");
  create_topic("/kitti/camera/color/left/image_raw", "sensor_msgs/msg/Image");
  create_topic("/kitti/camera/color/left/camera_info", "sensor_msgs/msg/CameraInfo");
  create_topic("/kitti/camera/color/right/image_raw", "sensor_msgs/msg/Image");
  create_topic("/kitti/camera/color/right/camera_info", "sensor_msgs/msg/CameraInfo");

  // OXTS topics
  create_topic("/kitti/oxts/gps/fix", "sensor_msgs/msg/NavSatFix");
  create_topic("/kitti/oxts/gps/vel", "geometry_msgs/msg/TwistStamped");
  create_topic("/kitti/oxts/imu", "sensor_msgs/msg/Imu");

  // Velodyne topic
  create_topic("/kitti/velo", "sensor_msgs/msg/PointCloud2");
}

void BagWriter::create_topic(const std::string & topic_name, const std::string & topic_type)
{
  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = topic_name;
  topic_metadata.type = topic_type;
  topic_metadata.serialization_format = "cdr";
  writer_->create_topic(topic_metadata);
}
