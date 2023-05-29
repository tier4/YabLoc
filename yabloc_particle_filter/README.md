# YabLoc Particle Filter

## Purpose

- This package provides the functionality of particle filter.
- A corrector provides only an abstract class and is intended to be inherited to implement a corrector node of your choice.
- A predictor can be used as is.

## How to test

```shell
colcon test --event-handlers console_cohesion+  --packages-select yabloc_particle_filter
colcon test-result --verbose --all
```

## Predictor

### Input

| Name                  | Type                                                   | Description                                               |
|-----------------------|--------------------------------------------------------|-----------------------------------------------------------|
| `/initialpose`        | `geometry_msgs::msg::PoseWithCovarianceStamped`        | to specity the initial position of particles              |
| `/twist`              | `geometry_msgs::msg::TwistStamped`                     | linear velocity and angular velocity of prediction update |
| `/twist_cov`          | `geometry_msgs::msg::TwistWithCovarianceStamped`       | linear velocity and angular velocity of prediction update |
| `/weighted_particles` | `yabloc_particle_filter::msg::ParticleArray` | particles weighted  by corrector nodes                    |
| `/height`             | `std_msgs::msg::Float32`                               | ground height                                             |

### Output

| Name                          | Type                                                   | Description                           |
|-------------------------------|--------------------------------------------------------|---------------------------------------|
| `/predicted_particles`        | `yabloc_particle_filter::msg::ParticleArray` | particles weighted by predictor nodes |
| `/predicted_particles_marker` | `visualization_msgs::msg::MarkerArray`                 | markers for particle visualization    |
| `/pose`                       | `geometry_msgs::msg::PoseStamped`                      | weighted mean of particles            |
| `/pose_with_covariance`       | `geometry_msgs::msg::PoseWithCovarianceStamped`        | weighted mean of particles            |

### Parameters

| Name                          | Type   | Default    | Description                                                                       |
|-------------------------------|--------|------------|-----------------------------------------------------------------------------------|
| `prediction_rate`             | double | 50         | frequency of forecast updates, in Hz                                              |
| `visualize`                   | bool   | false      | whether particles are also published in visualization_msgs or not                 |
| `num_of_particles`            | int    | 500        | the number of particles                                                           |
| `resampling_interval_seconds` | double | 1.0        | the interval of particle resamping                                                |
| `static_linear_covariance`    | double | 0.01       | to override the covariance of `/twist`. When using `/twist_cov`, it has no effect |
| `static_angular_covariance`   | double | 0.01       | to override the covariance of `/twist`. When using `/twist_cov`, it has no effect |
| `cov_xx_yy`   　　　　　　　　| list   | [2.0,0.25] | the covariance of initial pose                                                    |

## Corrector

### Input

| Name                   | Type                                                   | Description                               |
|------------------------|--------------------------------------------------------|-------------------------------------------|
| `/predicted_particles` | `yabloc_particle_filter::msg::ParticleArray` | particles predicted by the predictor node |

### Output

| Name                  | Type                                                   | Description                              |
|-----------------------|--------------------------------------------------------|------------------------------------------|
| `/weighted_particles` | `yabloc_particle_filter::msg::ParticleArray` | particles weighted by the corrector node |

### Parameters

| Name         | Type | Default | Description                                                       |
|--------------|------|---------|-------------------------------------------------------------------|
| `/visualize` | bool | false   | whether particles are also published in visualization_msgs or not |


# GNSS Particle Corrector

## Purpose

- Package for particle weighting using GNSS.
- It supports two types of input: `ublox_msgs::msg::NavPVT` and `geometry_msgs::msg::PoseWithCovarianceStamped`.

## Inputs / Outputs

### Input

| Name                    | Type                                                   | Description                              |
|-------------------------|--------------------------------------------------------|------------------------------------------|
| `/predicted_particles`  | `modularized_particle_filter_msgs::msg::ParticleArray` | predicted particles                      |
| `/pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped`        | pose measurement for weighting           |
| `/input/navpvt`         | `ublox_msgs::msg::NavPVT`                              | GNSS measurement for weighting           |
| `/input/height`         | `std_msgs::msg::Float32`                               | ground height used for gnss_range_marker |

### Output

| Name                  | Type                                                   | Description                         |
|-----------------------|--------------------------------------------------------|-------------------------------------|
| `/weighted_particles` | `modularized_particle_filter_msgs::msg::ParticleArray` | weighted particles                  |
| `/gnss_range_marker`  | `visualization_msgs::msg::MarkerArray`                 | visualized GNSS weight distribution |



### Parameters

| Name                              | Type  | Default | Description                                                                                           |
|-----------------------------------|-------|---------|-------------------------------------------------------------------------------------------------------|
| `/ignore_less_than_float`         | bool  | true    | if this is true, only FIX or FLOAT is used for correction (No effect when using pose_with_covariance) |
| `/mahalanobis_distance_threshold` | float | 20.f    | if the Mahalanobis distance to the GNSS for particle exceeds this, the correction skips.              |