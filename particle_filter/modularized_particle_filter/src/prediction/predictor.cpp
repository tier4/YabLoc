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

#include "modularized_particle_filter/prediction/predictor.hpp"

#include "modularized_particle_filter/common/mean.hpp"
#include "modularized_particle_filter/common/prediction_util.hpp"
#include "modularized_particle_filter/prediction/resampler.hpp"

#include <Eigen/Core>
#include <pcdless_common/pose_conversions.hpp>
#include <sophus/geometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <numeric>

namespace pcdless::modularized_particle_filter
{
Predictor::Predictor()
: Node("predictor"),
  number_of_particles_(declare_parameter("num_of_particles", 500)),
  resampling_interval_seconds_(declare_parameter("resampling_interval_seconds", 1.0f)),
  static_linear_covariance_(declare_parameter("static_linear_covariance", 0.01)),
  static_angular_covariance_(declare_parameter("static_angular_covariance", 0.01))
{
  tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Publishers
  predicted_particles_pub_ = create_publisher<ParticleArray>("predicted_particles", 10);
  pose_pub_ = create_publisher<PoseStamped>("pose", 10);
  pose_cov_pub_ = create_publisher<PoseCovStamped>("pose_with_covariance", 10);

  // Subscribers
  using std::placeholders::_1;
  auto on_initial = std::bind(&Predictor::on_initial_pose, this, _1);
  auto on_twist = std::bind(&Predictor::on_twist, this, _1);
  // clang-format off
  auto on_twist_cov= [this](const TwistCovStamped & twist_cov) -> void { this->latest_twist_opt_ = twist_cov; };
  // clang-format on
  auto on_particle = std::bind(&Predictor::on_weighted_particles, this, _1);
  auto on_height = [this](std_msgs::msg::Float32 m) -> void { this->ground_height_ = m.data; };

  initialpose_sub_ = create_subscription<PoseCovStamped>("initialpose", 1, on_initial);
  twist_sub_ = create_subscription<TwistStamped>("twist", 10, on_twist);
  twist_cov_sub_ = create_subscription<TwistCovStamped>("twist_cov", 10, on_twist_cov);
  particles_sub_ = create_subscription<ParticleArray>("weighted_particles", 10, on_particle);
  height_sub_ = create_subscription<std_msgs::msg::Float32>("height", 10, on_height);

  // Timer callback
  const double prediction_rate = declare_parameter("prediction_rate", 50.0f);
  auto on_timer = std::bind(&Predictor::on_timer, this);
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Rate(prediction_rate).period(), std::move(on_timer));

  // Optional modules
  if (declare_parameter<bool>("visualize", false)) {
    visualizer_ptr_ = std::make_unique<ParticleVisualizer>(*this);
  }
  if (declare_parameter("is_swap_mode", false)) {
    swap_mode_adaptor_ptr_ = std::make_unique<SwapModeAdaptor>(this);
  }
}

void Predictor::on_initial_pose(const PoseCovStamped::ConstSharedPtr initialpose)
{
  initialize_particles(*initialpose);
}

void Predictor::initialize_particles(const PoseCovStamped & initialpose)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "initialize_particles");
  modularized_particle_filter_msgs::msg::ParticleArray particle_array{};
  particle_array.header = initialpose.header;
  particle_array.id = 0;
  particle_array.particles.resize(number_of_particles_);

  Eigen::Matrix2d cov;
  cov(0, 0) = initialpose.pose.covariance[6 * 0 + 0];
  cov(0, 1) = initialpose.pose.covariance[6 * 0 + 1];
  cov(1, 0) = initialpose.pose.covariance[6 * 1 + 0];
  cov(1, 1) = initialpose.pose.covariance[6 * 1 + 1];

  const double yaw = tf2::getYaw(initialpose.pose.pose.orientation);
  for (auto & particle : particle_array.particles) {
    geometry_msgs::msg::Pose pose = initialpose.pose.pose;
    const Eigen::Vector2d noise = util::nrand_2d(cov);
    pose.position.x += noise.x();
    pose.position.y += noise.y();

    float noised_yaw =
      util::normalize_radian(yaw + util::nrand(sqrt(initialpose.pose.covariance[6 * 5 + 5])));
    pose.orientation.w = std::cos(noised_yaw / 2.0);
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(noised_yaw / 2.0);

    particle.pose = pose;
    particle.weight = 1.0;
  }
  particle_array_opt_ = particle_array;

  // We have to initialize resampler every particles initialization,
  // because resampler has particles resampling history and it will be outdate.
  resampler_ptr_ = std::make_unique<RetroactiveResampler>(number_of_particles_, 100);
}

void Predictor::on_twist(const TwistStamped::ConstSharedPtr twist)
{
  TwistCovStamped twist_covariance;
  twist_covariance.header = twist->header;
  twist_covariance.twist.twist = twist->twist;
  twist_covariance.twist.covariance.at(0) = static_linear_covariance_;
  twist_covariance.twist.covariance.at(7) = 1e4;
  twist_covariance.twist.covariance.at(14) = 1e4;
  twist_covariance.twist.covariance.at(21) = 1e4;
  twist_covariance.twist.covariance.at(28) = 1e4;
  twist_covariance.twist.covariance.at(35) = static_angular_covariance_;
  latest_twist_opt_ = twist_covariance;
}

void Predictor::update_with_dynamic_noise(
  ParticleArray & particle_array, const TwistCovStamped & twist, double dt)
{
  const float linear_x = twist.twist.twist.linear.x;
  const float angular_z = twist.twist.twist.angular.z;
  const float std_linear_x = std::sqrt(twist.twist.covariance[6 * 0 + 0]);
  const float std_angular_z = std::sqrt(twist.twist.covariance[6 * 5 + 5]);

  const float truncated_gain = std::clamp(std::sqrt(std::abs(linear_x)), 0.1f, 1.0f);
  const float truncated_linear_std = std::clamp(std_linear_x * linear_x, 0.1f, 2.0f);

  using util::nrand;
  for (auto & particle : particle_array.particles) {
    Sophus::SE3f se3_pose = common::pose_to_se3(particle.pose);
    Eigen::Matrix<float, 6, 1> noised_xi;
    noised_xi.setZero();
    noised_xi(0) = linear_x + nrand(truncated_linear_std);
    noised_xi(5) = angular_z + nrand(std_angular_z * truncated_gain);
    se3_pose *= Sophus::SE3f::exp(noised_xi * dt);

    geometry_msgs::msg::Pose pose = common::se3_to_pose(se3_pose);
    pose.position.z = ground_height_;
    particle.pose = pose;
  }
}

void Predictor::on_timer()
{
  // ==========================================================================
  // Pre-check section
  // TODO: Refactor
  if (swap_mode_adaptor_ptr_) {
    if (swap_mode_adaptor_ptr_->should_keep_update() == false) {
      return;
    }
    if (swap_mode_adaptor_ptr_->should_call_initialize()) {
      initialize_particles(swap_mode_adaptor_ptr_->init_pose());
    }
  }
  // Return if particle_array is not initialized yet
  if (!particle_array_opt_.has_value()) {
    return;
  }
  // Return if twist is not subscirbed yet
  if (!latest_twist_opt_.has_value()) {
    return;
  }
  //
  ParticleArray particle_array = particle_array_opt_.value();
  const rclcpp::Time current_time = this->now();
  const rclcpp::Time msg_time = particle_array.header.stamp;
  const double dt = (current_time - msg_time).seconds();
  particle_array.header.stamp = current_time;

  // ==========================================================================
  // Prediction section
  // NOTE: Sometimes particle_array.header.stamp is ancient due to lagged pose_initializer
  if (dt < 0.0 || dt > 1.0) {
    RCLCPP_WARN_STREAM(get_logger(), "time stamp is wrong? " << dt);
    return;
  }

  update_with_dynamic_noise(particle_array, latest_twist_opt_.value(), dt);

  // ==========================================================================
  // Post-process section
  //
  predicted_particles_pub_->publish(particle_array);
  //
  publish_mean_pose(mean_pose(particle_array), this->now());
  // If visualizer exists,
  if (visualizer_ptr_) {
    visualizer_ptr_->publish(particle_array);
  }

  particle_array_opt_ = particle_array;
}

void Predictor::on_weighted_particles(const ParticleArray::ConstSharedPtr weighted_particles_ptr)
{
  // NOTE: **We need not to check particle_array_opt.has_value().**
  // Since the weighted_particles is generated from messages published from this node,
  // the particle_array must have an entity in this function.
  ParticleArray particle_array = particle_array_opt_.value();

  // ==========================================================================
  // From here, weighting section
  particle_array =
    resampler_ptr_->add_weight_retroactively(particle_array, *weighted_particles_ptr);

  // ==========================================================================
  // From here, resampling section
  class resampling_skip_eception : public std::runtime_error
  {
  public:
    resampling_skip_eception(const char * message) : runtime_error(message) {}
  };

  const double current_time = rclcpp::Time(particle_array.header.stamp).seconds();
  try {
    // Exit if previous resampling time is not valid.
    if (!previous_resampling_time_opt_.has_value()) {
      previous_resampling_time_opt_ = current_time;
      throw resampling_skip_eception("previous resampling time is not valid");
    }

    if (current_time - previous_resampling_time_opt_.value() <= resampling_interval_seconds_) {
      throw resampling_skip_eception("it is not time to resample");
    }

    particle_array = resampler_ptr_->resample(particle_array);
    previous_resampling_time_opt_ = current_time;

  } catch (const resampling_skip_eception & e) {
    // Do nothing (just skipping the resample())
  }

  // ==========================================================================
  particle_array_opt_ = particle_array;
}

void Predictor::publish_mean_pose(
  const geometry_msgs::msg::Pose & mean_pose, const rclcpp::Time & stamp)
{
  // Publish pose
  {
    PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = mean_pose;
    pose_pub_->publish(pose_stamped);
  }

  // Publish pose with covariance
  {
    // TODO: Use particle distribution
    PoseCovStamped pose_cov_stamped;
    pose_cov_stamped.header.stamp = stamp;
    pose_cov_stamped.header.frame_id = "map";
    pose_cov_stamped.pose.pose = mean_pose;
    pose_cov_stamped.pose.covariance[6 * 0 + 0] = 0.255;
    pose_cov_stamped.pose.covariance[6 * 1 + 1] = 0.255;
    pose_cov_stamped.pose.covariance[6 * 2 + 2] = 0.255;
    pose_cov_stamped.pose.covariance[6 * 3 + 3] = 0.00625;
    pose_cov_stamped.pose.covariance[6 * 4 + 4] = 0.00625;
    pose_cov_stamped.pose.covariance[6 * 5 + 5] = 0.00625;

    pose_cov_pub_->publish(pose_cov_stamped);
  }

  // Publish TF
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = particle_array_opt_->header.stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "particle_filter";
    transform.transform.translation.x = mean_pose.position.x;
    transform.transform.translation.y = mean_pose.position.y;
    transform.transform.translation.z = mean_pose.position.z;
    transform.transform.rotation = mean_pose.orientation;
    tf2_broadcaster_->sendTransform(transform);
  }
}

}  // namespace pcdless::modularized_particle_filter