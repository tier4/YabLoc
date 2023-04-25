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

#include "modularized_particle_filter/prediction/resampler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/range/adaptor/indexed.hpp>

#include <cmath>
#include <numeric>

namespace pcdless::modularized_particle_filter
{
RetroactiveResampler::RetroactiveResampler(
  float resampling_interval_seconds, int number_of_particles, int max_history_num)
: resampling_interval_seconds_(resampling_interval_seconds),
  max_history_num_(max_history_num),
  number_of_particles_(number_of_particles),
  logger_(rclcpp::get_logger("modularized_particle_filter.retroactive_resampler")),
  resampling_history_(max_history_num_, number_of_particles)
{
  resampling_history_wp_ = 0;
}

RetroactiveResampler::OptParticleArray RetroactiveResampler::retroactive_weighting(
  const ParticleArray & predicted_particles,
  const ParticleArray::ConstSharedPtr & weighted_particles)
{
  assert(static_cast<int>(weighted_particles->particles.size()) == number_of_particles_);

  //
  if (!(weighted_particles->id <= resampling_history_wp_ &&                    // not future data
        weighted_particles->id > resampling_history_wp_ - max_history_num_ &&  // not old data
        weighted_particles->id >= 0))                                          // not error data
  {
    RCLCPP_WARN(logger_, "out of history");
    return std::nullopt;
  }

  ParticleArray reweighted_particles = predicted_particles;

  RCLCPP_INFO_STREAM(
    logger_, "current generation " << resampling_history_wp_ << " callback generation "
                                   << weighted_particles->id);

  // initialize corresponding index lookup table
  std::vector<int> index_table(weighted_particles->particles.size());
  std::iota(index_table.begin(), index_table.end(), 0);

  // lookup corresponding indices
  for (int history_wp{resampling_history_wp_}; history_wp > weighted_particles->id; history_wp--) {
    for (int m{0}; m < static_cast<int>(weighted_particles->particles.size()); m++) {
      if (
        0 <= index_table[m] &&
        index_table[m] < static_cast<int>(weighted_particles->particles.size())) {
        index_table[m] = resampling_history_[history_wp][index_table[m]];
      } else {
        return std::nullopt;
      }
    }
  }

  // Add weights to current particles
  float sum_weight = 0;
  for (auto && it : reweighted_particles.particles | boost::adaptors::indexed()) {
    it.value().weight *= weighted_particles->particles[index_table[it.index()]].weight;
    sum_weight += it.value().weight;
  }

  // Normalize all weight
  for (auto & particle : reweighted_particles.particles) {
    particle.weight /= sum_weight;
  }

  return reweighted_particles;
}

RetroactiveResampler::OptParticleArray RetroactiveResampler::resampling(
  const ParticleArray & predicted_particles)
{
  const double current_time = rclcpp::Time(predicted_particles.header.stamp).seconds();

  // Exit if previous resampling time is not valid.
  if (!previous_resampling_time_opt_.has_value()) {
    previous_resampling_time_opt_ = current_time;
    return std::nullopt;
  }

  //
  if (current_time - previous_resampling_time_opt_.value() <= resampling_interval_seconds_) {
    return std::nullopt;
  }

  ParticleArray resampled_particles{predicted_particles};
  resampling_history_wp_++;
  resampled_particles.id = resampling_history_wp_;

  //
  const double sum_weight = std::accumulate(
    predicted_particles.particles.begin(), predicted_particles.particles.end(), 0.0,
    [](double weight, const Particle & ps) { return weight + ps.weight; });
  //
  const double sum_weight_inv = 1.0 / sum_weight;
  //
  const double num_of_particles_inv = 1.0 / static_cast<double>(number_of_particles_);
  //
  const double r = rand() / static_cast<double>(RAND_MAX) * num_of_particles_inv;

  if (!std::isfinite(sum_weight_inv)) {
    RCLCPP_ERROR_STREAM(logger_, "The inverse of the sum of the weights is not a valid value");
    exit(EXIT_FAILURE);
  }

  double c = predicted_particles.particles[0].weight * sum_weight_inv;

  int i = 0;
  for (int m{0}; m < static_cast<int>(predicted_particles.particles.size()); m++) {
    const double u{r + m * num_of_particles_inv};

    while (c < u) {
      i++;
      c += predicted_particles.particles[i].weight * sum_weight_inv;
    }

    resampled_particles.particles[m] = predicted_particles.particles[i];
    resampled_particles.particles[m].weight = num_of_particles_inv;  // TODO:
    resampling_history_[resampling_history_wp_][m] = i;
  }

  previous_resampling_time_opt_ = current_time;

  return resampled_particles;
}

}  // namespace pcdless::modularized_particle_filter