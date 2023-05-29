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
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace yabloc
{
class LaneImage
{
public:
  using Pose = geometry_msgs::msg::Pose;
  using SharedPtr = std::shared_ptr<LaneImage>;
  LaneImage(lanelet::LaneletMapPtr map);

  cv::Mat get_image(const Pose & pose);

  cv::Mat create_vectormap_image(const Eigen::Vector3f & position);

private:
  lanelet::LaneletMapPtr map_;
};
}  // namespace yabloc