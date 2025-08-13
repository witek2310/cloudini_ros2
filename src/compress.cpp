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
#include <filesystem>
#include <fstream>

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
  : Node("CloudiniROS2")
  {
    this->declare_parameter<std::string>("PC_topic", "/velodyne/velodyne_points");
    this->declare_parameter<double>("resolution", 0.001);
    this->declare_parameter<std::string>("csv_folder_path", "/tmp");

    _resolution = this->get_parameter("resolution").as_double();



    std::string pc_topic;
    this->get_parameter("PC_topic", pc_topic);
    RCLCPP_INFO(this->get_logger(), "Subscribing to point cloud topic: %s", pc_topic.c_str());
    
    RCLCPP_INFO(this->get_logger(), "Using resolution: %f", _resolution);
    
    _compressed_pc_publisher = this->create_publisher<cloudini_ros2::msg::CompressedPointCloud>("compressed_pc", 10);
    _pc_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pc_topic, 10, std::bind(&CloudiniROS2::pc_callback, this, std::placeholders::_1));
    
    std::string folder_path;
    this->get_parameter("csv_folder_path", folder_path);
      
    _csv_file_path = folder_path + "/compress_cloudini.csv";
    std::cout << _csv_file_path << std::endl;
    RCLCPP_INFO(this->get_logger(), "saved dataunder: %s", _csv_file_path.c_str());


    if (!std::filesystem::exists(_csv_file_path)) {
      std::filesystem::create_directories(std::filesystem::path(_csv_file_path).parent_path()); 
      std::ofstream file(_csv_file_path);
      if (file.is_open()) {
        file << "points_number,point_cloud_size,compresion_time,size_after_compresion\n";  // Header row
        file.close();
      }
    }
  }

private:
  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr raw)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud with width: %u", raw->width);


    auto start_compresion = std::chrono::steady_clock::now();
    auto info = ConvertToEncodingInfo(raw, _resolution);
    Cloudini::PointcloudEncoder encoder(info);

    cloudini_ros2::msg::CompressedPointCloud compressed_msg;
    compressed_msg.header = raw->header;


    compressed_msg.data.resize(raw->data.size()); // reserve space for compressed data

    Cloudini::ConstBufferView input(raw->data.data(), raw->data.size());

    auto new_size = encoder.encode(input, compressed_msg.data);
    compressed_msg.data.resize(new_size); // resize to the actual size of compressed data
    
    // publish the transformed message
    auto end_compresion = std::chrono::steady_clock::now();
    auto elapsed_compresion = std::chrono::duration_cast<std::chrono::microseconds>(end_compresion - start_compresion).count();
    _compressed_pc_publisher->publish(compressed_msg);  

    std::ofstream ofs(_csv_file_path, std::ios_base::app);
    if (!ofs) {
      std::filesystem::create_directories(std::filesystem::path(_csv_file_path).parent_path()); 
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", _csv_file_path.c_str());
      return;
    }
    ofs << raw->width * raw->height << "," << raw->row_step * raw->height << ","<< elapsed_compresion << "," << new_size << "\n";
    ofs.close();
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _pc_subscription;
  rclcpp::Publisher<cloudini_ros2::msg::CompressedPointCloud>::SharedPtr _compressed_pc_publisher;
  double _resolution = 0.001; // Default resolution for compression
  std::string _csv_file_path;
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