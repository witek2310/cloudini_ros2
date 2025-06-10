// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "cloudini_ros2/msg/compressed_point_cloud.hpp"

#include <cloudini_lib/cloudini.hpp>
// #include <cloudini_ros/conversion_utils.hpp>
// #include <cloudini_ros/conversion_utils.hpp>
// #include "conversion_utils.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

Cloudini::EncodingInfo ConvertToEncodingInfo(const sensor_msgs::msg::PointCloud2::SharedPtr msg, float resolution);

class CloudiniROS2 : public rclcpp::Node
{
public:
  CloudiniROS2()
  : Node("CloudiniROS2"), count_(0)
  {
    this->declare_parameter<std::string>("PC_topic", "/velodyne/velodyne_points");
    this->declare_parameter<double>("resolution", 0.001);

    this->get_parameter("resolution", resolution_);


    std::string pc_topic;
    this->get_parameter("PC_topic", pc_topic);
    RCLCPP_INFO(this->get_logger(), "Subscribing to point cloud topic: %s", pc_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Using resolution: %f", resolution_);

    _compressed_pc_publisher_ = this->create_publisher<cloudini_ros2::msg::CompressedPointCloud>("compressed_pc", 10);
    _pc_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pc_topic, 10, std::bind(&CloudiniROS2::pc_callback, this, std::placeholders::_1));
  }

private:
  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr raw)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud with width: %u", raw->width);

    // Here you would implement the compression logic
    // For demonstration, we will just create an empty CompressedPointCloud message

    auto info = ConvertToEncodingInfo(raw, resolution_);
    Cloudini::PointcloudEncoder encoder(info);

    cloudini_ros2::msg::CompressedPointCloud compressed_msg;
    compressed_msg.header = raw->header;


    compressed_msg.data.resize(raw->data.size()); // reserve space for compressed data

    Cloudini::ConstBufferView input(raw->data.data(), raw->data.size());

    auto new_size = encoder.encode(input, compressed_msg.data);
    compressed_msg.data.resize(new_size); // resize to the actual size of compressed data
    
    // publish the transformed message
    _compressed_pc_publisher_->publish(compressed_msg);  
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _pc_subscription;
  rclcpp::Publisher<cloudini_ros2::msg::CompressedPointCloud>::SharedPtr _compressed_pc_publisher_;
  double resolution_ = 0.001; // Default resolution for compression
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudiniROS2>());
  rclcpp::shutdown();
  return 0;
}


Cloudini::EncodingInfo ConvertToEncodingInfo(const sensor_msgs::msg::PointCloud2::SharedPtr msg, float resolution) {
  Cloudini::EncodingInfo info;
  info.width = msg->width;
  info.height = msg->height;
  info.point_step = msg->point_step;
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
  info.compression_opt = Cloudini::CompressionOption::ZSTD;

  for (const auto& msg_field : msg->fields) {
    Cloudini::PointField field;
    field.name = msg_field.name;
    field.offset = msg_field.offset;
    field.type = static_cast<Cloudini::FieldType>(msg_field.datatype);
    field.resolution = (field.type == Cloudini::FieldType::FLOAT32) ? std::optional<float>(resolution) : std::nullopt;
    info.fields.push_back(field);
  }
  return info;
}

// CloudiniPublisher::TypedEncodeResult CloudiniPublisher::encodeTyped(const sensor_msgs::msg::PointCloud2& raw) const {
//   auto info = Cloudini::ConvertToEncodingInfo(raw, resolution_);
//   Cloudini::PointcloudEncoder encoder(info);

//   // copy all the fields from the raw point cloud to the compressed one
//   point_cloud_interfaces::msg::CompressedPointCloud2 result;

//   result.header = raw.header;
//   result.width = raw.width;
//   result.height = raw.height;
//   result.fields = raw.fields;
//   result.is_bigendian = false;
//   result.point_step = raw.point_step;
//   result.row_step = raw.row_step;
//   result.is_dense = raw.is_dense;

//   // reserve memory for the compressed data
//   result.compressed_data.resize(raw.data.size());

//   // prepare buffer for compression
//   Cloudini::ConstBufferView input(raw.data.data(), raw.data.size());
//   auto new_size = encoder.encode(input, result.compressed_data);

//   // resize the compressed data to the actual size
//   result.compressed_data.resize(new_size);
//   return result;
// }