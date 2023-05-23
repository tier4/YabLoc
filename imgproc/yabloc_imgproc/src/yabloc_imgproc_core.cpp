#include "yabloc_imgproc/yabloc_imgproc.hpp"

#include <yabloc_common/pub_sub.hpp>

#include <cv_bridge/cv_bridge.h>

namespace yabloc
{

ImageProcessingNode::ImageProcessingNode()
: Node("image_processing"), override_frame_id_(declare_parameter<std::string>("override_frame_id"))
{
  using std::placeholders::_1;

  rclcpp::QoS qos{10};
  if (declare_parameter<bool>("use_sensor_qos")) {
    qos = rclcpp::QoS(10).durability_volatile().best_effort();
  }

  undistort_module_ = std::make_unique<Undistort>(this);

  auto on_image = std::bind(&ImageProcessingNode::on_compressed_image, this, _1);
  auto on_info = std::bind(&ImageProcessingNode::on_info, this, _1);
  sub_image_ = create_subscription<CompressedImage>("src_image", qos, std::move(on_image));
  sub_info_ = create_subscription<CameraInfo>("src_info", qos, std::move(on_info));

  pub_info_ = create_publisher<CameraInfo>("resized_info", 10);
  pub_image_ = create_publisher<Image>("resized_image", 10);
}

void ImageProcessingNode::on_compressed_image(const CompressedImage image_msg)
{
  if (!info_.has_value()) return;

  auto [scaled_image, scaled_info] = undistort_module_->undistort(image_msg, info_.value());

  // Publish CameraInfo
  {
    scaled_info.header = info_->header;
    if (override_frame_id_ != "") scaled_info.header.frame_id = override_frame_id_;
    pub_info_->publish(scaled_info);
  }

  // Publish Image
  {
    cv_bridge::CvImage bridge;
    bridge.header.stamp = image_msg.header.stamp;
    if (override_frame_id_ != "")
      bridge.header.frame_id = override_frame_id_;
    else
      bridge.header.frame_id = image_msg.header.frame_id;
    bridge.encoding = "bgr8";
    bridge.image = scaled_image;
    pub_image_->publish(*bridge.toImageMsg());
  }
}

}  // namespace yabloc