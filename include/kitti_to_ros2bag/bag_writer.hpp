#pragma once

// C++ standard library
#include <string>
#include <memory>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>


class BagWriter
{
public:
  BagWriter(const std::string & output_bag_name);

  void create_topics();

  template<typename T>
  void write_message(const T & msg, const std::string & topic_name, const rclcpp::Time & timestamp)
  {
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<T> serializer;
    serializer.serialize_message(&msg, &serialized_msg);

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_message->topic_name = topic_name;
    bag_message->recv_timestamp = timestamp.nanoseconds();

    auto & rcl_serialized_msg = serialized_msg.get_rcl_serialized_message();
    bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>();
    *bag_message->serialized_data = rcl_serialized_msg;

    writer_->write(bag_message);
  }

private:
  void create_topic(const std::string & topic_name, const std::string & topic_type);
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};
