#pragma once

// C++ standard library
#include <string>
#include <memory>
#include <vector>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosidl_runtime_cpp/traits.hpp>


class BagWriter
{
public:
  explicit BagWriter(const std::string & output_bag_name);

  void create_topics();

  template<typename T>
  void write_message(const T & msg, const std::string & topic_name, const rclcpp::Time & timestamp)
  {
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<T> serializer;
    serializer.serialize_message(&msg, serialized_msg.get());

    // Use the Writer::write() overload that accepts rclcpp::SerializedMessage directly.
    // This avoids the old pattern of manually constructing SerializedBagMessage and copying
    // the raw rcutils_uint8_array_t buffer (which left a dangling pointer when the
    // SerializedMessage went out of scope).
    writer_->write(
      serialized_msg,
      topic_name,
      rosidl_generator_traits::name<T>(),
      timestamp);
  }

private:
  void create_topic(
    const std::string & topic_name,
    const std::string & topic_type,
    const std::vector<rclcpp::QoS> & qos_profiles);
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};
