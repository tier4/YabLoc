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
#include "yabloc_imgproc/graph_segment.hpp"
#include "yabloc_imgproc/line_segment_detector.hpp"
#include "yabloc_imgproc/segment_filter.hpp"
#include "yabloc_imgproc/undistort.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace yabloc
{
class ImageProcessingNode : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  ImageProcessingNode();

private:
  const std::string override_frame_id_;

  std::unique_ptr<Undistort> undistort_module_{nullptr};
  std::unique_ptr<LineSegmentDetector> lsd_module_{nullptr};
  std::unique_ptr<GraphSegment> graph_module_{nullptr};
  std::unique_ptr<SegmentFilter> filter_module_{nullptr};

  std::optional<CameraInfo> info_{std::nullopt};

  rclcpp::Subscription<CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<CameraInfo>::SharedPtr pub_info_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_with_line_segments_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<Image>::SharedPtr pub_debug_image_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_projected_cloud_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_debug_cloud_;
  rclcpp::Publisher<Image>::SharedPtr pub_projected_image_;

  void on_compressed_image(const CompressedImage image_msg);
  void on_info(const CameraInfo & info_msg) { info_ = info_msg; }

  void publish_overriding_frame_id(
    const CompressedImage & image_msg, const cv::Mat & scaled_image,
    const CameraInfo & scaled_info);
};
}  // namespace yabloc