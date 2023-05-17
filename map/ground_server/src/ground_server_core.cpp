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

#include "ground_server/ground_server.hpp"
#include "ground_server/polygon_operation.hpp"
#include "ground_server/util.hpp"

#include <Eigen/Eigenvalues>
#include <ll2_decomposer/from_bin_msg.hpp>
#include <yabloc_common/color.hpp>
#include <yabloc_common/pub_sub.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace yabloc ::ground_server
{
GroundServer::GroundServer()
: Node("ground_server"),
  force_zero_tilt_(declare_parameter<bool>("force_zero_tilt")),
  R(declare_parameter<int>("R")),
  K(declare_parameter<int>("K"))
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  auto on_pose = std::bind(&GroundServer::on_pose_stamped, this, _1);
  auto on_map = std::bind(&GroundServer::on_map, this, _1);
  auto on_service = std::bind(&GroundServer::on_service, this, _1, _2);

  service_ = create_service<Ground>("ground", on_service);

  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, on_map);
  sub_pose_stamped_ = create_subscription<PoseStamped>("particle_pose", 10, on_pose);

  pub_ground_height_ = create_publisher<Float32>("height", 10);
  pub_ground_plane_ = create_publisher<Float32Array>("ground", 10);
  pub_marker_ = create_publisher<Marker>("ground_marker", 10);
  pub_string_ = create_publisher<String>("ground_status", 10);
  pub_near_cloud_ = create_publisher<PointCloud2>("near_cloud", 10);
}

void GroundServer::on_initial_pose(const PoseCovStamped & msg)
{
  if (kdtree_ == nullptr) {
    RCLCPP_FATAL_STREAM(get_logger(), "ground height is not initialized because map is empty");
    return;
  }

  const Point point = msg.pose.pose.position;
  height_filter_.initialize(estimate_height_simply(point));
}

void GroundServer::on_pose_stamped(const PoseStamped & msg)
{
  if (kdtree_ == nullptr) return;
  GroundPlane ground_plane = estimate_ground(msg.pose.position);
  // Publish value msg
  Float32 data;
  data.data = ground_plane.height();
  pub_ground_height_->publish(data);
  pub_ground_plane_->publish(ground_plane.msg());

  // Publish string msg
  {
    std::stringstream ss;
    ss << "--- Ground Estimator Status ----" << std::endl;
    ss << std::fixed << std::setprecision(2);
    ss << "height: " << ground_plane.height() << std::endl;
    float cos = ground_plane.normal.dot(Eigen::Vector3f::UnitZ());
    ss << "tilt: " << std::acos(cos) * 180 / 3.14 << " deg" << std::endl;

    String string_msg;
    string_msg.data = ss.str();
    pub_string_->publish(string_msg);
  }

  // Publish nearest point cloud for debug
  {
    pcl::PointCloud<pcl::PointXYZ> near_cloud;
    for (int index : last_indices_) {
      near_cloud.push_back(cloud_->at(index));
    }
    if (!near_cloud.empty()) common::publish_cloud(*pub_near_cloud_, near_cloud, msg.header.stamp);
  }
}

void GroundServer::on_map(const HADMapBin & msg)
{
  lanelet::LaneletMapPtr lanelet_map = ll2_decomposer::from_bin_msg(msg);

  // TODO: has to be loaded from rosparm
  const std::set<std::string> visible_labels = {
    "zebra_marking",      "virtual",   "line_thin", "line_thick",
    "pedestrian_marking", "stop_line", "curbstone"};

  pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_cloud =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  for (lanelet::LineString3d & line : lanelet_map->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;

    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;

    lanelet::ConstPoint3d const * from = nullptr;
    for (const lanelet::ConstPoint3d & p : line) {
      if (from != nullptr) upsample_line_string(*from, p, upsampled_cloud);
      from = &p;
    }
  }

  // NOTE: Under construction
  // if (lanelet_map->polygonLayer.size() > 0)
  //   *upsampled_cloud += sample_from_polygons(lanelet_map->polygonLayer);

  cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(upsampled_cloud);
  filter.setLeafSize(1.0f, 1.0f, 1.0f);
  filter.filter(*cloud_);

  kdtree_ = pcl::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  kdtree_->setInputCloud(cloud_);
}

float GroundServer::estimate_height_simply(const geometry_msgs::msg::Point & point) const
{
  // NOTE: Sometimes it might give not-accurate height
  constexpr float sq_radius = 3.0 * 3.0;
  const float x = point.x;
  const float y = point.y;

  float height = std::numeric_limits<float>::infinity();
  for (const auto & p : cloud_->points) {
    const float dx = x - p.x;
    const float dy = y - p.y;
    const float sd = (dx * dx) + (dy * dy);
    if (sd < sq_radius) {
      height = std::min(height, static_cast<float>(p.z));
    }
  }
  return std::isfinite(height) ? height : 0;
}

std::vector<int> GroundServer::estimate_inliers_by_ransac(const std::vector<int> & indices_raw)
{
  pcl::PointIndicesPtr indices(new pcl::PointIndices);
  indices->indices = indices_raw;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(1.0);
  seg.setProbability(0.6);

  seg.setInputCloud(cloud_);
  seg.setIndices(indices);
  seg.segment(*inliers, *coefficients);
  return inliers->indices;
}

GroundServer::GroundPlane GroundServer::estimate_ground(const Point & point)
{
  const float predicted_z = height_filter_.get_estimate();
  const pcl::PointXYZ xyz(point.x, point.y, predicted_z);

  std::vector<int> raw_indices;
  std::vector<float> distances;
  kdtree_->nearestKSearch(xyz, K, raw_indices, distances);

  std::vector<int> indices = estimate_inliers_by_ransac(raw_indices);

  if (indices.empty()) indices = raw_indices;
  last_indices_ = indices;

  // Estimate normal vector using covariance matrix around the target point
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_, indices, centroid);
  pcl::computeCovarianceMatrix(*cloud_, indices, centroid, covariance);

  // NOTE: I forgot why I don't use coefficients computeed by SACSegmentation
  Eigen::Vector4f plane_parameter;
  float curvature;
  pcl::solvePlaneParameters(covariance, centroid, plane_parameter, curvature);
  Eigen::Vector3f normal = plane_parameter.topRows(3).normalized();

  {
    // Reverse if it is upside down
    if (normal.z() < 0) normal = -normal;

    // Remove NaN
    if (!normal.allFinite()) {
      normal = Eigen::Vector3f::UnitZ();
      RCLCPP_WARN_STREAM(get_logger(), "Reject NaN tilt");
    }
    // Remove too large tilt (0.707 = cos(45deg))
    if ((normal.dot(Eigen::Vector3f::UnitZ())) < 0.707) {
      normal = Eigen::Vector3f::UnitZ();
      RCLCPP_WARN_STREAM(get_logger(), "Reject too large tilt of ground");
    }
  }

  const Eigen::Vector3f filt_normal = normal_filter_.update(normal);

  GroundPlane plane;
  plane.xyz = Eigen::Vector3f(point.x, point.y, predicted_z);
  plane.normal = filt_normal;

  // Compute z value by intersection of estimated plane and orthogonal line
  {
    Eigen::Vector3f center = centroid.topRows(3);
    float inner = center.dot(plane.normal);
    float px_nx = point.x * plane.normal.x();
    float py_ny = point.y * plane.normal.y();
    plane.xyz.z() = (inner - px_nx - py_ny) / plane.normal.z();
  }

  height_filter_.update(plane.xyz.z());

  if (force_zero_tilt_) plane.normal = Eigen::Vector3f::UnitZ();
  return plane;
}

void GroundServer::on_service(
  const std::shared_ptr<Ground::Request> request, std::shared_ptr<Ground::Response> response)
{
  if (kdtree_ == nullptr) return;
  float z = estimate_height_simply(request->point);
  response->pose.position.x = request->point.x;
  response->pose.position.y = request->point.y;
  response->pose.position.z = z;
}

void GroundServer::publish_marker(const GroundPlane & plane)
{
  // TODO: current visual marker is so useless
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.type = Marker::TRIANGLE_LIST;
  marker.id = 0;
  marker.color = common::Color(0.0, 0.5, 0.0, 0.1);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.pose.position.x = plane.xyz.x();
  marker.pose.position.y = plane.xyz.y();
  marker.pose.position.z = plane.xyz.z();

  Eigen::Vector3f n = plane.normal;
  const int L = 20;
  auto toPoint = [](float x, float y, float z) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  };
  auto p1 = toPoint(L, 0, -L * n.x() / n.z());
  auto p2 = toPoint(0, L, -L * n.y() / n.z());
  auto p3 = toPoint(-L, 0, L * n.x() / n.z());
  auto p4 = toPoint(0, -L, L * n.y() / n.z());

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p3);

  marker.points.push_back(p3);
  marker.points.push_back(p4);
  marker.points.push_back(p1);
  pub_marker_->publish(marker);
}

}  // namespace yabloc::ground_server
