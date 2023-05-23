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
#include "yabloc_imgproc/graph_segment/similar_area_searcher.hpp"

#include <opencv4/opencv2/ximgproc/segmentation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace yabloc
{
class GraphSegment
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  GraphSegment(rclcpp::Node * node);

  std::pair<cv::Mat, cv::Mat> execute(const cv::Mat & image);

private:
  const float target_height_ratio_;
  const int target_candidate_box_width_;

  cv::Ptr<cv::ximgproc::segmentation::GraphSegmentation> segmentation_;
  std::unique_ptr<SimilarAreaSearcher> similar_area_searcher_{nullptr};

  int search_most_road_like_class(const cv::Mat & segmented) const;

  cv::Mat draw_image(const cv::Mat & raw_image, const cv::Mat & debug_image);
};
}  // namespace yabloc