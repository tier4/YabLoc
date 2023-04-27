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

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <set>
#include <map>
#include <memory>
#include <rclcpp/time.hpp>

class ImageLineDrawer
{
public:
  ImageLineDrawer(size_t max_buffer_size) : max_buffer_size_(max_buffer_size) {}

  void set_new_image(const sensor_msgs::msg::Image::ConstSharedPtr& image_ptr)
  {
    rclcpp::Time timestamp(image_ptr->header.stamp);
    image_buffer_[timestamp] = image_ptr;

    if (image_buffer_.size() > max_buffer_size_) {
      image_buffer_.erase(image_buffer_.begin());
    }
  }

  cv::Mat create_debug_image(
    const rclcpp::Time& timestamp,
    const pcl::PointCloud<pcl::PointNormal>::Ptr& line_segments_cloud,
    const std::set<int>& indices)
  {
    auto iter = image_buffer_.find(timestamp);
    // If the iterator reaches the end of the image_buffer_, it means the image with the given timestamp
    // is not found in the buffer. This assumption is based on the fact that images are received and
    // stored in the buffer BEFORE their corresponding line segments are processed. If this assumption
    // does not hold, the function may throw a runtime error indicating that the image with the given
    // timestamp was not found.
    if (iter == image_buffer_.end())
    {
      throw std::runtime_error("Image with the given timestamp not found.");
    }

    auto image_ptr = iter->second;
    cv::Mat image = cv_bridge::toCvShare(image_ptr, "bgr8")->image;

    for (size_t index = 0; index < line_segments_cloud->size(); ++index) {
      const pcl::PointNormal& pn = line_segments_cloud->at(index);
      Eigen::Vector3f xy1 = pn.getVector3fMap();
      Eigen::Vector3f xy2 = pn.getNormalVector3fMap();

      cv::Scalar color(0, 0, 255); // Red
      if (indices.count(index) > 0) {
        color = cv::Scalar(0, 255, 0); // Green
      }
      
      cv::line(image, cv::Point(xy1(0), xy1(1)), cv::Point(xy2(0), xy2(1)), color, 2);
    }

    return image;
  }

private:
  std::map<rclcpp::Time, sensor_msgs::msg::Image::ConstSharedPtr> image_buffer_;
  size_t max_buffer_size_;
};