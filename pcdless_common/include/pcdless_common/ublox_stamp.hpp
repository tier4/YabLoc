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
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace pcdless::common
{
rclcpp::Time ublox_time_to_stamp(const ublox_msgs::msg::NavPVT & msg);
ublox_msgs::msg::NavPVT stamp_to_ublox_time(const builtin_interfaces::msg::Time & stamp);

}  // namespace pcdless::common