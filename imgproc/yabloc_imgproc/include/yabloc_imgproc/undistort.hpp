// Copyright 2023 TIER IV, Inc.
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

#pragma once

#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <optional>

namespace yabloc
{
class Undistort
{
public:
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using CompressedImage = sensor_msgs::msg::CompressedImage;

  Undistort(rclcpp::Node * node) : output_width_(node->declare_parameter<int>("width")) {}

  std::pair<cv::Mat, CameraInfo> undistort(
    const CompressedImage & image_msg, const CameraInfo & info_msg);

private:
  const int output_width_;

  std::optional<CameraInfo> scaled_info_{std::nullopt};
  cv::Mat undistort_map_x, undistort_map_y;

  void make_remap_lut(const CameraInfo & info_msg);
};
}  // namespace yabloc