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

#include "yabloc_imgproc/line_segment_detector.hpp"

#include <opencv4/opencv2/imgproc.hpp>
#include <yabloc_common/cv_decompress.hpp>
#include <yabloc_common/pub_sub.hpp>
#include <yabloc_common/timer.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace yabloc
{
LineSegmentDetector::LineSegmentDetector()
{
  using std::placeholders::_1;

  line_segment_detector_ =
    cv::createLineSegmentDetector(cv::LSD_REFINE_STD, 0.8, 0.6, 2.0, 22.5, 0, 0.7, 1024);
}

std::pair<cv::Mat, pcl::PointCloud<pcl::PointNormal>> LineSegmentDetector::execute(
  const cv::Mat & image)
{
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

  cv::Mat lines;
  {
    common::Timer timer;
    line_segment_detector_->detect(gray_image, lines);
    line_segment_detector_->drawSegments(gray_image, lines);
  }

  pcl::PointCloud<pcl::PointNormal> line_cloud;
  std::vector<cv::Mat> filtered_lines = remove_too_outer_elements(lines, image.size());

  for (const cv::Mat & xy_xy : filtered_lines) {
    Eigen::Vector3f xy1, xy2;
    xy1 << xy_xy.at<float>(0), xy_xy.at<float>(1), 0;
    xy2 << xy_xy.at<float>(2), xy_xy.at<float>(3), 0;
    pcl::PointNormal pn;
    pn.getVector3fMap() = xy1;
    pn.getNormalVector3fMap() = xy2;
    line_cloud.push_back(pn);
  }

  return {gray_image, line_cloud};
}

std::vector<cv::Mat> LineSegmentDetector::remove_too_outer_elements(
  const cv::Mat & lines, const cv::Size & size) const
{
  std::vector<cv::Rect2i> rect_vector;
  rect_vector.emplace_back(0, 0, size.width, 3);
  rect_vector.emplace_back(0, size.height - 3, size.width, 3);
  rect_vector.emplace_back(0, 0, 3, size.height);
  rect_vector.emplace_back(size.width - 3, 0, 3, size.height);

  std::vector<cv::Mat> filtered_lines;
  for (int i = 0; i < lines.rows; i++) {
    cv::Mat xy_xy = lines.row(i);
    cv::Point2f xy1(xy_xy.at<float>(0), xy_xy.at<float>(1));
    cv::Point2f xy2(xy_xy.at<float>(2), xy_xy.at<float>(3));

    bool enabled = true;
    for (const cv::Rect2i & rect : rect_vector) {
      if (rect.contains(xy1) && rect.contains(xy2)) {
        enabled = false;
        break;
      }
    }
    if (enabled) filtered_lines.push_back(xy_xy);
  }

  return filtered_lines;
}

}  // namespace yabloc
