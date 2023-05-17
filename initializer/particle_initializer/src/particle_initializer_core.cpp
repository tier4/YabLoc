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

#include "particle_initializer/particle_initializer.hpp"

namespace yabloc::modularized_particle_filter
{
ParticleInitializer::ParticleInitializer()
: Node("particle_initializer"),
  cov_xx_yy_{this->template declare_parameter<std::vector<double>>("cov_xx_yy")}
{
  using std::placeholders::_1;

  // Publisher
  pub_initialpose_ = create_publisher<PoseCovStamped>("rectified/initialpose", 10);
  pub_marker_ = create_publisher<Marker>("init/marker", 10);

  // Subscriber
  auto on_initial_pose = std::bind(&ParticleInitializer::on_initial_pose, this, _1);
  sub_initialpose_ =
    create_subscription<PoseCovStamped>("initialpose3d", 10, std::move(on_initial_pose));
}

void ParticleInitializer::on_initial_pose(const PoseCovStamped & initialpose)
{
  auto position = initialpose.pose.pose.position;
  Eigen::Vector3f pos_vec3f;
  pos_vec3f << position.x, position.y, position.z;

  auto orientation = initialpose.pose.pose.orientation;
  float theta = 2 * std::atan2(orientation.z, orientation.w);
  Eigen::Vector3f tangent;
  tangent << std::cos(theta), std::sin(theta), 0;

  publish_range_marker(pos_vec3f, tangent);

  pub_initialpose_->publish(rectify_initial_pose(pos_vec3f, tangent, initialpose));
}

void ParticleInitializer::publish_range_marker(
  const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent)
{
  Marker msg;
  msg.type = Marker::LINE_STRIP;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "map";
  msg.scale.x = 0.2;
  msg.scale.y = 0.2;

  msg.color.r = 0;
  msg.color.g = 1;
  msg.color.b = 0;
  msg.color.a = 1;

  auto cast2gp = [](const Eigen::Vector3f & vec3f) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point p;
    p.x = vec3f.x();
    p.y = vec3f.y();
    p.z = vec3f.z();
    return p;
  };

  Eigen::Vector3f binormal;
  binormal << -tangent.y(), tangent.x(), tangent.z();

  msg.points.push_back(cast2gp(pos + tangent + binormal));
  msg.points.push_back(cast2gp(pos + tangent - binormal));
  msg.points.push_back(cast2gp(pos - tangent - binormal));
  msg.points.push_back(cast2gp(pos - tangent + binormal));
  msg.points.push_back(cast2gp(pos + tangent + binormal));

  pub_marker_->publish(msg);
}

ParticleInitializer::PoseCovStamped ParticleInitializer::rectify_initial_pose(
  const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent,
  const PoseCovStamped & raw_initialpose)
{
  PoseCovStamped msg = raw_initialpose;
  msg.header.frame_id = "map";
  msg.pose.pose.position.x = pos.x();
  msg.pose.pose.position.y = pos.y();
  msg.pose.pose.position.z = pos.z();

  float theta = std::atan2(tangent.y(), tangent.x());

  msg.pose.pose.orientation.w = std::cos(theta / 2);
  msg.pose.pose.orientation.x = 0;
  msg.pose.pose.orientation.y = 0;
  msg.pose.pose.orientation.z = std::sin(theta / 2);

  Eigen::Matrix2f cov;
  cov << cov_xx_yy_.at(0), 0, 0, cov_xx_yy_.at(1);
  Eigen::Rotation2D r(theta);
  cov = r * cov * r.inverse();

  msg.pose.covariance.at(6 * 0 + 0) = cov(0, 0);
  msg.pose.covariance.at(6 * 0 + 1) = cov(0, 1);
  msg.pose.covariance.at(6 * 1 + 0) = cov(1, 0);
  msg.pose.covariance.at(6 * 1 + 1) = cov(1, 1);
  msg.pose.covariance.at(6 * 5 + 5) = 0.0076;  // 0.0076 = (5deg)^2

  return msg;
}

}  // namespace yabloc::modularized_particle_filter