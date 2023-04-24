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

#ifndef IMAGE_OVERLAY_DISPLAY_HPP_
#define IMAGE_OVERLAY_DISPLAY_HPP_

#include <rclcpp/qos.hpp>

#include <memory>
#include <mutex>

#ifndef Q_MOC_RUN
#include "jsk_overlay_utils.hpp"

#include <image_transport/image_transport.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#endif

namespace rviz_plugins
{
class ImageOverlayDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  ImageOverlayDisplay();
  ~ImageOverlayDisplay() override;

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;
  void subscribe();
  void unsubscribe();

private Q_SLOTS:
  void updateVisualization();
  void updateQoS();

protected:
  void update(float wall_dt, float ros_dt) override;
  void processMessage(const sensor_msgs::msg::Image::ConstSharedPtr msg_ptr);
  void processCompressedMessage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg_ptr);

  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_width_;
  rviz_common::properties::IntProperty * property_height_;
  rviz_common::properties::StringProperty * property_topic_name_;
  rviz_common::properties::FloatProperty * property_alpha_;
  rviz_common::properties::BoolProperty * property_image_type_;
  rviz_common::properties::EnumProperty * property_qos_reliability_;
  rviz_common::properties::EnumProperty * property_qos_durability_;
  rclcpp::QoS custom_qos_profile_;

private:
  sensor_msgs::msg::Image::ConstSharedPtr last_msg_ptr_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_;
  std::string topic_name_;
  bool update_required_ = true;
};

}  // namespace rviz_plugins

#endif  // IMAGE_OVERLAY_DISPLAY_HPP_
