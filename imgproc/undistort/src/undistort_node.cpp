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

#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <pcdless_common/cv_decompress.hpp>
#include <pcdless_common/pub_sub.hpp>
#include <pcdless_common/timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <optional>

namespace pcdless::undistort
{
class UndistortNode : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Image = sensor_msgs::msg::Image;

  UndistortNode()
  : Node("undistort"),
    OUTPUT_WIDTH(declare_parameter("width", 800)),
    OVERRIDE_FRAME_ID(declare_parameter("override_frame_id", ""))
  {
    using std::placeholders::_1;

    rclcpp::QoS qos{10};
    if (declare_parameter("use_sensor_qos", false)) {
      qos = rclcpp::QoS(10).durability_volatile().best_effort();
    }

    auto on_image = std::bind(&UndistortNode::on_image, this, _1);
    auto on_info = std::bind(&UndistortNode::on_info, this, _1);
    sub_image_ = create_subscription<CompressedImage>("src_image", qos, std::move(on_image));
    sub_info_ = create_subscription<CameraInfo>("src_info", qos, std::move(on_info));

    pub_info_ = create_publisher<CameraInfo>("resized_info", 10);
    pub_image_ = create_publisher<Image>("resized_image", 10);
  }

private:
  const int OUTPUT_WIDTH;
  const std::string OVERRIDE_FRAME_ID;

  rclcpp::Subscription<CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<CameraInfo>::SharedPtr pub_info_;
  std::optional<CameraInfo> info_{std::nullopt};
  std::optional<CameraInfo> scaled_info_{std::nullopt};

  cv::Mat undistort_map_x, undistort_map_y;

  void make_remap_lut()
  {
    if (!info_.has_value()) return;
    cv::Mat K = cv::Mat(cv::Size(3, 3), CV_64FC1, (void *)(info_->k.data()));
    cv::Mat D = cv::Mat(cv::Size(5, 1), CV_64FC1, (void *)(info_->d.data()));
    cv::Size size(info_->width, info_->height);

    cv::Size new_size = size;
    if (OUTPUT_WIDTH > 0)
      new_size = cv::Size(OUTPUT_WIDTH, 1.0f * OUTPUT_WIDTH / size.width * size.height);

    cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, D, size, 0, new_size);

    cv::initUndistortRectifyMap(
      K, D, cv::Mat(), new_K, new_size, CV_32FC1, undistort_map_x, undistort_map_y);

    scaled_info_ = sensor_msgs::msg::CameraInfo{};
    scaled_info_->k.at(0) = new_K.at<double>(0, 0);
    scaled_info_->k.at(2) = new_K.at<double>(0, 2);
    scaled_info_->k.at(4) = new_K.at<double>(1, 1);
    scaled_info_->k.at(5) = new_K.at<double>(1, 2);
    scaled_info_->k.at(8) = 1;
    scaled_info_->d.resize(5);
    scaled_info_->width = new_size.width;
    scaled_info_->height = new_size.height;
  }

  void on_image(const CompressedImage & msg)
  {
    if (!info_.has_value()) return;
    if (undistort_map_x.empty()) make_remap_lut();

    common::Timer timer;
    cv::Mat image = common::decompress_to_cv_mat(msg);

    cv::Mat undistorted_image;
    cv::remap(image, undistorted_image, undistort_map_x, undistort_map_y, cv::INTER_LINEAR);

    // Publish CameraInfo
    {
      scaled_info_->header = info_->header;
      if (OVERRIDE_FRAME_ID != "") scaled_info_->header.frame_id = OVERRIDE_FRAME_ID;
      pub_info_->publish(scaled_info_.value());
    }

    // Publish Image
    {
      cv_bridge::CvImage bridge;
      bridge.header.stamp = msg.header.stamp;
      if (OVERRIDE_FRAME_ID != "")
        bridge.header.frame_id = OVERRIDE_FRAME_ID;
      else
        bridge.header.frame_id = msg.header.frame_id;
      bridge.encoding = "bgr8";
      bridge.image = undistorted_image;
      pub_image_->publish(*bridge.toImageMsg());
    }

    RCLCPP_INFO_STREAM(get_logger(), "image undistort: " << timer);
  }

  void on_info(const CameraInfo & msg) { info_ = msg; }
};
}  // namespace pcdless::undistort

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::undistort::UndistortNode>());
  rclcpp::shutdown();
  return 0;
}
