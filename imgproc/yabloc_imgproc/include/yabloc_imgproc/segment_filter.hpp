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
#include <yabloc_common/static_tf_subscriber.hpp>
#include <yabloc_common/synchro_subscriber.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace yabloc
{
class SegmentFilter
{
public:
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  SegmentFilter(rclcpp::Node * node);

  std::tuple<pcl::PointCloud<pcl::PointXYZLNormal>, pcl::PointCloud<pcl::PointXYZLNormal>, cv::Mat>
  execute(const pcl::PointCloud<pcl::PointNormal> & msg1, const cv::Mat & msg2);
  void set_info(const CameraInfo & info) { info_ = info; }

private:
  using ProjectFunc = std::function<std::optional<Eigen::Vector3f>(const Eigen::Vector3f &)>;
  const int image_size_;
  const float max_range_;
  const float min_segment_length_;
  const float max_segment_distance_;
  const float max_lateral_distance_;

  std::optional<CameraInfo> info_{std::nullopt};
  common::StaticTfSubscriber tf_subscriber_;

  ProjectFunc project_func_ = nullptr;

  // Return true if success to define or already defined
  bool define_project_func();

  pcl::PointCloud<pcl::PointNormal> project_lines(
    const pcl::PointCloud<pcl::PointNormal> & lines, const std::set<int> & indices,
    bool negative = false) const;

  std::set<int> filt_by_mask(const cv::Mat & mask, const pcl::PointCloud<pcl::PointNormal> & edges);

  cv::Point2i to_cv_point(const Eigen::Vector3f & v) const;

  bool is_near_element(const pcl::PointNormal & pn, pcl::PointNormal & truncated_pn) const;
};
}  // namespace yabloc