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

#include "graph_segment/graph_segment.hpp"
#include "graph_segment/histogram.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <pcdless_common/cv_decompress.hpp>
#include <pcdless_common/pub_sub.hpp>
#include <pcdless_common/timer.hpp>

namespace pcdless::graph_segment
{
GraphSegment::GraphSegment()
: Node("graph_segment"),
  target_height_ratio_(declare_parameter<float>("target_height_ratio", 0.85)),
  target_candidate_box_width_(declare_parameter<int>("target_candidate_box_width", 15))
{
  using std::placeholders::_1;

  // Subscriber
  sub_image_ =
    create_subscription<Image>("src_image", 10, std::bind(&GraphSegment::on_image, this, _1));

  pub_mask_image_ = create_publisher<Image>("mask_image", 10);
  pub_debug_image_ = create_publisher<Image>("segmented_image", 10);

  const double sigma = declare_parameter<double>("sigma", 0.5);
  const float k = declare_parameter<float>("k", 300);
  const int min_size = declare_parameter<double>("min_size", 100);
  segmentation_ = cv::ximgproc::segmentation::createGraphSegmentation(sigma, k, min_size);

  // additional area pickup module
  if (declare_parameter<bool>("pickup_additional_areas", true)) {
    similar_area_searcher_ = std::make_unique<SimilarAreaSearcher>(
      declare_parameter<float>("similarity_score_threshold", 0.8));
  }
}

cv::Vec3b random_hsv(int index)
{
  double base = (double)(index)*0.7071;
  return cv::Vec3b(fmod(base, 1.2) * 255, 0.7 * 255, 0.5 * 255);
};

int GraphSegment::search_most_road_like_class(const cv::Mat & segmented) const
{
  const int W = target_candidate_box_width_;
  const float R = target_height_ratio_;
  cv::Point2i target_px(segmented.cols * 0.5, segmented.rows * R);
  cv::Rect2i rect(target_px + cv::Point2i(-W, -W), target_px + cv::Point2i(W, W));

  std::unordered_map<int, int> areas;
  std::unordered_set<int> candidates;
  for (int h = 0; h < segmented.rows; h++) {
    const int * seg_ptr = segmented.ptr<int>(h);
    for (int w = 0; w < segmented.cols; w++) {
      int key = seg_ptr[w];
      if (areas.count(key) == 0) areas[key] = 0;
      areas[key]++;
      if (rect.contains(cv::Point2i{w, h})) candidates.insert(key);
    }
  }

  // Search the largest area and its class
  int max_area = 0;
  int max_area_class = -1;
  for (int c : candidates) {
    if (areas.at(c) < max_area) continue;
    max_area = areas.at(c);
    max_area_class = c;
  }
  return max_area_class;
}

void GraphSegment::on_image(const Image & msg)
{
  cv::Mat image = common::decompress_to_cv_mat(msg);
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);

  // Execute graph-based segmentation
  common::Timer timer;
  cv::Mat segmented;
  segmentation_->processImage(resized, segmented);
  RCLCPP_INFO_STREAM(get_logger(), "segmentation time: " << timer);

  //
  int target_class = search_most_road_like_class(segmented);
  //
  std::set<int> road_keys = {target_class};
  if (similar_area_searcher_) {
    road_keys = similar_area_searcher_->search(resized, segmented, target_class);
  }

  // Draw output image and debug image
  // TODO: use ptr instead of at()
  cv::Mat output_image = cv::Mat::zeros(resized.size(), CV_8UC1);
  cv::Mat debug_image = cv::Mat::zeros(resized.size(), CV_8UC3);
  for (int h = 0; h < resized.rows; h++) {
    for (int w = 0; w < resized.cols; w++) {
      cv::Point2i px(w, h);
      int key = segmented.at<int>(px);
      if (road_keys.count(key) > 0) {
        output_image.at<uchar>(px) = 255;
        if (key == target_class)
          debug_image.at<cv::Vec3b>(px) = cv::Vec3b(30, 255, 255);
        else
          debug_image.at<cv::Vec3b>(px) = cv::Vec3b(10, 255, 255);
      } else {
        debug_image.at<cv::Vec3b>(px) = random_hsv(key);
      }
    }
  }
  cv::cvtColor(debug_image, debug_image, cv::COLOR_HSV2BGR);
  cv::resize(output_image, output_image, image.size(), 0, 0, cv::INTER_NEAREST);
  cv::resize(debug_image, debug_image, image.size(), 0, 0, cv::INTER_NEAREST);

  common::publish_image(*pub_mask_image_, output_image, msg.header.stamp);

  draw_and_publish_image(image, debug_image, msg.header.stamp);
  RCLCPP_INFO_STREAM(get_logger(), "total processing time: " << timer);
}

void GraphSegment::draw_and_publish_image(
  const cv::Mat & raw_image, const cv::Mat & debug_image, const rclcpp::Time & stamp)
{
  cv::Mat show_image;
  cv::addWeighted(raw_image, 0.5, debug_image, 0.8, 1.0, show_image);
  const cv::Size size = debug_image.size();

  // Draw target rectangle
  {
    const int W = target_candidate_box_width_;
    const float R = target_height_ratio_;
    cv::Point2i target(size.width / 2, size.height * R);
    cv::Rect2i rect(target + cv::Point2i(-W, -W), target + cv::Point2i(W, W));
    cv::rectangle(show_image, rect, cv::Scalar::all(0), 2);
  }

  common::publish_image(*pub_debug_image_, show_image, stamp);
}

}  // namespace pcdless::graph_segment