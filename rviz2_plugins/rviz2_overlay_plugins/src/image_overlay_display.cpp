// Copyright 2023 Tier IV, Inc.
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

// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Software License Agreement (BSD License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.S SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#include "image_overlay_display.hpp"

#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <rviz_common/uniform_string_stream.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <cv_bridge/cv_bridge.h>

namespace rviz_plugins
{
ImageOverlayDisplay::ImageOverlayDisplay() : custom_qos_profile_(10)
{
  property_topic_name_ = new rviz_common::properties::StringProperty(
    "Topic", "/", "String", this, SLOT(updateVisualization()));
  property_left_ = new rviz_common::properties::IntProperty(
    "Left", 128, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", 128, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_width_ = new rviz_common::properties::IntProperty(
    "Width", 256, "Width of the plotter window", this, SLOT(updateVisualization()), this);
  property_width_->setMin(10);
  property_height_ = new rviz_common::properties::IntProperty(
    "Height", 256, "Height of the plotter window", this, SLOT(updateVisualization()), this);
  property_height_->setMin(10);

  property_alpha_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.8, "Foreground Alpha", this, SLOT(updateVisualization()), this);
  property_alpha_->setMin(0.0);
  property_alpha_->setMax(1.0);

  property_image_type_ = new rviz_common::properties::BoolProperty(
    "Image Topic Style", true, "is compresed?", this, SLOT(updateVisualization()));

  property_qos_reliability_ = new rviz_common::properties::EnumProperty(
    "QoS Reliability", "Reliable", "Select the desired QoS reliability policy", this,
    SLOT(updateQoS()));

  property_qos_reliability_->addOption("Reliable", 0);
  property_qos_reliability_->addOption("Best Effort", 1);

  property_qos_durability_ = new rviz_common::properties::EnumProperty(
    "QoS Durability", "Volatile", "Select the desired QoS durability policy", this,
    SLOT(updateQoS()));

  property_qos_durability_->addOption("Volatile", 0);
  property_qos_durability_->addOption("Transient Local", 1);
}

ImageOverlayDisplay::~ImageOverlayDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void ImageOverlayDisplay::updateQoS()
{
  custom_qos_profile_ = rclcpp::QoS(10);  // 10 is the depth of the history cache

  // Set the reliability policy
  if (property_qos_reliability_->getOptionInt() == 0)
    custom_qos_profile_.reliable();
  else
    custom_qos_profile_.best_effort();

  // Set the durability policy
  if (property_qos_durability_->getOptionInt() == 0)
    custom_qos_profile_.durability_volatile();
  else
    custom_qos_profile_ = custom_qos_profile_.transient_local();

  // Resubscribe to the topic with the updated QoS settings
  subscribe();
}

void ImageOverlayDisplay::onInitialize()
{
  static int count = 0;

  rviz_common::UniformStringStream ss;
  ss << "ImageOverlayDisplay" << count++;
  auto logger = context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger();
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(scene_manager_, logger, ss.str()));

  overlay_->show();

  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
}

void ImageOverlayDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void ImageOverlayDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void ImageOverlayDisplay::subscribe()
{
  topic_name_ = property_topic_name_->getStdString();
  std::cout << "try to subscribe " << topic_name_ << std::endl;
  if (topic_name_.length() > 0 && topic_name_ != "/") {
    auto raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    sub_ = raw_node->create_subscription<sensor_msgs::msg::Image>(
      topic_name_, custom_qos_profile_,
      std::bind(&ImageOverlayDisplay::processMessage, this, std::placeholders::_1));
  }
}

void ImageOverlayDisplay::unsubscribe() { sub_.reset(); }

void ImageOverlayDisplay::processMessage(const sensor_msgs::msg::Image::ConstSharedPtr msg_ptr)
{
  if (!isEnabled()) return;

  last_msg_ptr_ = msg_ptr;
  update_required_ = true;
  queueRender();
}

void ImageOverlayDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  std::string current_topic_name = property_topic_name_->getStdString();
  if (current_topic_name != topic_name_) {
    topic_name_ = current_topic_name;
    subscribe();
  }

  if (update_required_) {
    updateVisualization();
    update_required_ = false;
  }
}

void ImageOverlayDisplay::updateVisualization()
{
  if (!last_msg_ptr_) return;
  if (last_msg_ptr_->width == 0 || last_msg_ptr_->height == 0) return;

  overlay_->updateTextureSize(last_msg_ptr_->width, last_msg_ptr_->height);
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(property_width_->getInt(), property_height_->getInt());

  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);

  cv::Mat bgr_image;
  try {
    const cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(last_msg_ptr_, sensor_msgs::image_encodings::BGR8);
    bgr_image = cv_ptr->image;
  } catch (cv_bridge::Exception & e) {
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
  }

  const int w = hud.width();
  const int h = hud.height();

  std::vector<cv::Mat> channels;
  cv::split(bgr_image, channels);
  const cv::Mat a_image(
    bgr_image.rows, bgr_image.cols, CV_8UC1, cv::Scalar(property_alpha_->getFloat() * 255.0));
  channels.push_back(a_image);

  cv::Mat bgra_image;
  cv::merge(channels, bgra_image);

  memcpy(hud.scanLine(0), bgra_image.data, w * h * bgra_image.elemSize());
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ImageOverlayDisplay, rviz_common::Display)
