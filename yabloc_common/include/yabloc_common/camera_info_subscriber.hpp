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
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <optional>
namespace yabloc::common
{
class CameraInfoSubscriber
{
public:
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  CameraInfoSubscriber(rclcpp::Node * node);

  bool is_camera_info_ready() const;

  bool is_camera_info_nullopt() const;

  Eigen::Vector2i size() const;

  Eigen::Matrix3f intrinsic() const;

  // This member function DOES NOT check isCameraInfoReady()
  // If it it not ready, throw bad optional access
  std::string get_frame_id() const;

private:
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  std::optional<CameraInfo> opt_info_;
};
}  // namespace yabloc::common