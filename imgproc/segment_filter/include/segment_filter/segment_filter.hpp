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
#include <pcdless_common/camera_info_subscriber.hpp>
#include <pcdless_common/static_tf_subscriber.hpp>
#include <pcdless_common/synchro_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::segment_filter
{
class SegmentFilter : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;

  SegmentFilter();

private:
  using ProjectFunc = std::function<std::optional<Eigen::Vector3f>(const Eigen::Vector3f &)>;
  const int image_size_;
  const float max_range_;
  const float min_segment_length_;
  const float max_segment_distance_;
  const float max_lateral_distance_;

  common::CameraInfoSubscriber info_;
  common::SynchroSubscriber<PointCloud2, Image> synchro_subscriber_;
  common::StaticTfSubscriber tf_subscriber_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_middle_cloud_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;

  ProjectFunc project_func_ = nullptr;

  // Return true if success to define or already defined
  bool define_project_func();

  pcl::PointCloud<pcl::PointNormal> project_lines(
    const pcl::PointCloud<pcl::PointNormal> & lines, const std::set<int> & indices,
    bool negative = false) const;

  std::set<int> filt_by_mask(const cv::Mat & mask, const pcl::PointCloud<pcl::PointNormal> & edges);

  cv::Point2i to_cv_point(const Eigen::Vector3f & v) const;
  void execute(const PointCloud2 & msg1, const Image & msg2);

  bool is_near_element(const pcl::PointNormal & pn, pcl::PointNormal & truncated_pn) const;
};
}  // namespace pcdless::segment_filter