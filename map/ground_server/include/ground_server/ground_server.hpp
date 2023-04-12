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
#include "ground_server/filter/low_pass_filter.hpp"
#include "ground_server/filter/moving_averaging.hpp"

#include <pcdless_common/ground_plane.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ground_msgs/srv/ground.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::ground_server
{
class GroundServer : public rclcpp::Node
{
public:
  using GroundPlane = common::GroundPlane;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Ground = ground_msgs::srv::Ground;

  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  using Float32 = std_msgs::msg::Float32;
  using Float32Array = std_msgs::msg::Float32MultiArray;  // TODO: define new msg
  using Marker = visualization_msgs::msg::Marker;
  using String = std_msgs::msg::String;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Point = geometry_msgs::msg::Point;
  GroundServer();

private:
  const bool force_zero_tilt_;
  const float R;
  const int K;

  // Service
  rclcpp::Service<Ground>::SharedPtr service_;
  // Subscriber
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_stamped_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_initial_pose_;
  // Publisher
  rclcpp::Publisher<Float32>::SharedPtr pub_ground_height_;
  rclcpp::Publisher<Float32Array>::SharedPtr pub_ground_plane_;
  rclcpp::Publisher<Marker>::SharedPtr pub_marker_;
  rclcpp::Publisher<String>::SharedPtr pub_string_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_near_cloud_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{nullptr};
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};

  // Smoother
  MovingAveraging normal_filter_;
  LowPassFilter height_filter_;

  // For debug
  std::vector<int> last_indices_;

  // Callback
  void on_map(const HADMapBin & msg);
  void on_initial_pose(const PoseCovStamped & msg);
  void on_pose_stamped(const PoseStamped & msg);
  void on_service(
    const std::shared_ptr<Ground::Request> request, std::shared_ptr<Ground::Response> response);

  // Body
  GroundPlane estimate_ground(const Point & point);

  // Return inlier indices which are belong to a plane
  // Sometimes, this return empty indices due to RANSAC failure
  std::vector<int> estimate_inliers_by_ransac(const std::vector<int> & indices_raw);

  // Return the lowest point's height around given point
  float estimate_height_simply(const Point & point) const;

  // Visualize estimated ground as plane
  void publish_marker(const GroundPlane & plane);
};

}  // namespace pcdless::ground_server