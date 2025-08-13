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



Cloudini::EncodingInfo ConvertToEncodingInfo(const sensor_msgs::msg::PointCloud2::SharedPtr msg, float resolution);

class CloudiniROS2 : public rclcpp::Node
{
public:
  CloudiniROS2()
  : Node("CloudiniROS2")
  {
    this->declare_parameter<std::string>("csv_folder_path", "/tmp");

    _compressed_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/decompressed_cloudini", 10);
    _pc_subscription = this->create_subscription<cloudini_ros2::msg::CompressedPointCloud>(
      "compressed_pc", 10, std::bind(&CloudiniROS2::pc_callback, this, std::placeholders::_1));
    decoder_ = std::make_shared<Cloudini::PointcloudDecoder>();


    std::string folder_path;
    this->get_parameter("csv_folder_path", folder_path);
    _csv_file_path = folder_path + "/decompress_cloudini.csv";

    if (!std::filesystem::exists(_csv_file_path)) {
      std::filesystem::create_directories(std::filesystem::path(_csv_file_path).parent_path()); 
      std::ofstream file(_csv_file_path);
      if (file.is_open()) {
        file << "time_stamp,points_number_after_decompression,decompresion_time,size_before_decompression\n";
        file.close();
      }
    }
  }

private:
  void pc_callback(const cloudini_ros2::msg::CompressedPointCloud::SharedPtr msg){
    if (msg->data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Received empty compressed point cloud data!");
        return;
    }


    auto start = std::chrono::high_resolution_clock::now();
    Cloudini::ConstBufferView input(msg->data.data(), msg->data.size());
    auto info = Cloudini::DecodeHeader(input);

    // Create the output message
    auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();
    result->header = msg->header;
    result->height = info.height;
    result->width = info.width;
    result->is_bigendian = false;
    result->point_step = info.point_step;
    result->row_step = info.point_step * info.width;
    result->is_dense = true; // or info.is_dense, if available

    // Populate fields from info.fields
    result->fields.resize(info.fields.size());
    for (size_t i = 0; i < info.fields.size(); ++i) {
        const auto& field = info.fields[i];
        auto& result_field = result->fields[i];
        result_field.name = field.name;
        result_field.offset = field.offset;
        result_field.datatype = static_cast<uint8_t>(field.type);
        result_field.count = 1; // Or field.count if available
    }

    // Allocate the output buffer
    result->data.resize(result->point_step * result->width * result->height);
    Cloudini::BufferView output(result->data.data(), result->data.size());

    decoder_->decode(info, input, output);

    auto end = std::chrono::high_resolution_clock::now();
    
    _compressed_pc_publisher->publish(*result);

    std::chrono::duration<double, std::milli> decompression_time = end - start;

    std::ofstream ofs(_csv_file_path, std::ios_base::app);
    if (!ofs) {
      std::filesystem::create_directories(std::filesystem::path(_csv_file_path).parent_path()); 
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", _csv_file_path.c_str());
      return;
    }
    ofs << result->width * result->height << "," << decompression_time.count() << "," << msg->data.size() <<"\n";
    ofs.close();
}

  std::shared_ptr<Cloudini::PointcloudDecoder> decoder_ = std::make_shared<Cloudini::PointcloudDecoder>();
  rclcpp::Subscription<cloudini_ros2::msg::CompressedPointCloud>::SharedPtr _pc_subscription;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _compressed_pc_publisher;
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