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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>

namespace yabloc
{
class LineSegmentDetector
{
public:
  LineSegmentDetector();

  std::pair<cv::Mat, pcl::PointCloud<pcl::PointNormal>> execute(const cv::Mat & image);

private:
  cv::Ptr<cv::LineSegmentDetector> line_segment_detector_;

  std::vector<cv::Mat> remove_too_outer_elements(
    const cv::Mat & lines, const cv::Size & size) const;
  void on_image(const sensor_msgs::msg::Image & msg);
};
}  // namespace yabloc
