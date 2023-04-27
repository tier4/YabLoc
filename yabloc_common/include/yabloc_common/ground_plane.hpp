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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sophus/geometry.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

namespace yabloc::common
{
struct GroundPlane
{
  using Float32Array = std_msgs::msg::Float32MultiArray;
  Eigen::Vector3f xyz;
  Eigen::Vector3f normal;

  GroundPlane()
  {
    xyz.setZero();
    normal = Eigen::Vector3f::UnitZ();
  }
  GroundPlane(const Float32Array & array) { set(array); }

  void set(const Float32Array & array)
  {
    if (array.data.size() != 6) exit(EXIT_FAILURE);
    for (int i = 0; i < 3; i++) {
      xyz(i) = array.data.at(i);
      normal(i) = array.data.at(3 + i);
    }
  }

  float height() const { return xyz.z(); }

  Eigen::Quaternionf align_with_slope(const Eigen::Quaternionf & q) const
  {
    return Eigen::Quaternionf{align_with_slope(q.toRotationMatrix())};
  }

  Eigen::Matrix3f align_with_slope(const Eigen::Matrix3f & R) const
  {
    Eigen::Matrix3f R_;
    Eigen::Vector3f rz = this->normal;
    Eigen::Vector3f azimuth = R * Eigen::Vector3f::UnitX();
    Eigen::Vector3f ry = (rz.cross(azimuth)).normalized();
    Eigen::Vector3f rx = ry.cross(rz);
    R_.col(0) = rx;
    R_.col(1) = ry;
    R_.col(2) = rz;
    return R_;
  }

  Sophus::SE3f align_with_slope(const Sophus::SE3f & pose) const
  {
    return {align_with_slope(pose.rotationMatrix()), pose.translation()};
  }

  Eigen::Affine3f align_with_slope(const Eigen::Affine3f & pose) const
  {
    Eigen::Matrix3f R = pose.rotation();
    Eigen::Vector3f t = pose.translation();
    return Eigen::Translation3f(t) * align_with_slope(R);
  }

  Float32Array msg() const
  {
    Float32Array array;
    for (int i = 0; i < 3; i++) array.data.push_back(xyz(i));
    for (int i = 0; i < 3; i++) array.data.push_back(normal(i));
    return array;
  }
};
}  // namespace yabloc::common