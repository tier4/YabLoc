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
#include <rclcpp/rclcpp.hpp>

namespace pcdless::modularized_particle_filter
{
struct WeightManager
{
  struct Parameter
  {
    float flat_radius_;
    float max_radius_;
    float max_weight_;
    float min_weight_;

    float coeff_;
    void compute_coeff()
    {
      coeff_ = -std::log(min_weight_ / max_weight_) / (max_radius_ * max_radius_);
    }
  };

  Parameter for_fixed_;
  Parameter for_not_fixed_;

  WeightManager(rclcpp::Node * node)
  {
    for_fixed_.flat_radius_ = node->declare_parameter("for_fixed/flat_radius", 0.5f);
    for_fixed_.max_radius_ = node->declare_parameter("for_fixed/max_radius", 10.0f);
    for_fixed_.min_weight_ = node->declare_parameter("for_fixed/min_weight", 0.5f);
    for_fixed_.max_weight_ = node->declare_parameter("for_fixed/max_weight", 1.5f);
    for_fixed_.compute_coeff();

    for_not_fixed_.flat_radius_ = node->declare_parameter("for_not_fixed/flat_radius", 5.0f);
    for_not_fixed_.max_radius_ = node->declare_parameter("for_not_fixed/max_radius", 20.0f);
    for_not_fixed_.min_weight_ = node->declare_parameter("for_not_fixed/min_weight", 0.5f);
    for_not_fixed_.max_weight_ = node->declare_parameter("for_not_fixed/max_weight", 1.0f);
    for_not_fixed_.compute_coeff();
  }

  float normal_pdf(float distance, const Parameter & param) const
  {
    // NOTE: This is not exact normal distribution because of no scale factor depending on sigma
    float d = std::clamp(std::abs(distance) - param.flat_radius_, 0.f, param.max_radius_);
    return param.max_weight_ * std::exp(-param.coeff_ * d * d);
  }

  float normal_pdf(float distance, bool is_rtk_fixed) const
  {
    if (is_rtk_fixed) {
      return normal_pdf(distance, for_fixed_);
    } else {
      return normal_pdf(distance, for_not_fixed_);
    }
  }

  float inverse_normal_pdf(float prob, const Parameter & param) const
  {
    prob = (param.max_weight_ - param.min_weight_) * prob + param.min_weight_;

    if (prob > param.max_radius_) return param.max_radius_;
    if (prob < param.min_weight_) return param.flat_radius_ + param.max_radius_;
    return param.flat_radius_ + std::sqrt(-std::log(prob / param.max_weight_) / param.coeff_);
  }

  float inverse_normal_pdf(float prob, bool is_rtk_fixed) const
  {
    if (is_rtk_fixed) {
      return inverse_normal_pdf(prob, for_fixed_);
    } else {
      return inverse_normal_pdf(prob, for_not_fixed_);
    }
  }
};
}  // namespace pcdless::modularized_particle_filter