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

#include "refine_optimizer/refine.hpp"
#include "refine_optimizer/util.hpp"

#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <sophus/geometry.hpp>
#include <vml_common/cv_decompress.hpp>
#include <vml_common/pose_conversions.hpp>
#include <vml_common/pub_sub.hpp>

#include <boost/functional/hash.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace refine_optimizer
{
RefineOptimizer::RefineOptimizer()
: Node("refine"),
  pixel_interval_(declare_parameter<int>("refine.pixel_interval", 10)),
  show_grad_image_(declare_parameter<bool>("refine.show_grad_image", false)),
  pose_buffer_{40},
  tf_subscriber_(get_clock())
{
  using std::placeholders::_1, std::placeholders::_2;

  auto cb_synchro = std::bind(&RefineOptimizer::imageAndLsdCallback, this, _1, _2);
  using MySynchroSub = SynchroSubscriber<Image, PointCloud2>;
  sub_synchro_ = std::make_shared<MySynchroSub>(this, "/src_image", "/line_segments_cloud");
  sub_synchro_->setCallback(cb_synchro);

  // Subscriber
  auto cb_info = std::bind(&RefineOptimizer::infoCallback, this, _1);
  auto cb_pose = [this](const PoseStamped & msg) -> void { pose_buffer_.push_back(msg); };
  auto cb_ground = [this](const Float32Array & msg) -> void { ground_plane_.set(msg); };

  sub_ground_plane_ = create_subscription<Float32Array>("/ground", 10, cb_ground);
  sub_pose_ = create_subscription<PoseStamped>("/particle_pose", 10, cb_pose);
  sub_info_ = create_subscription<CameraInfo>("/src_info", 10, cb_info);
  sub_ll2_ = create_subscription<PointCloud2>(
    "/ll2_road_marking", 10,
    [this](const PointCloud2 & msg) -> void { pcl::fromROSMsg(msg, ll2_cloud_); });

  // Publisher
  pub_image_ = create_publisher<Image>("/refine_image", 10);
  pub_pose_ = create_publisher<PoseStamped>("/refine_pose", 10);
  pub_pose_cov_stamped_ = create_publisher<PoseCovStamped>("/refine_pose_with_covariance", 10);
  pub_string_ = create_publisher<String>("/refine_string", 10);

  optimizer_ = std::make_shared<Optimizer>(RefineConfig{this});

  const float gamma = declare_parameter<float>("refine.gamma", 5);
  gamma_converter_.reset(gamma);
}

void RefineOptimizer::infoCallback(const CameraInfo & msg)
{
  info_ = msg;
  camera_extrinsic_ = tf_subscriber_.se3f(info_->header.frame_id, "base_link");
}

void RefineOptimizer::imageAndLsdCallback(const Image & image_msg, const PointCloud2 & line_segments_msg)
{
  const rclcpp::Time stamp = line_segments_msg.header.stamp;

  // Search synchronized pose
  float min_dt = std::numeric_limits<float>::max();
  geometry_msgs::msg::PoseStamped synched_pose;
  for (auto pose : pose_buffer_) {
    auto dt = (rclcpp::Time(pose.header.stamp) - stamp);
    auto abs_dt = std::abs(dt.seconds());
    if (abs_dt < min_dt) {
      min_dt = abs_dt;
      synched_pose = pose;
    }
  }
  if (min_dt > 0.1) return;

  LineSegments line_segments;
  pcl::fromROSMsg(line_segments_msg, line_segments);
  cv::Mat cost_image = makeCostMap(line_segments);

  Sophus::SE3f raw_pose = vml_common::pose2Se3(synched_pose.pose);
  auto line_segments = extractNaerLineSegments(raw_pose, ll2_cloud_);
  pcl::PointCloud<pcl::PointXYZ> samples = sampleUniformlyOnImage(raw_pose, line_segments);

  Sophus::SE3f opt_pose = raw_pose;
  std::string summary_text;
  {
    // Optimization
    Eigen::Matrix3f K =
      Eigen::Map<Eigen::Matrix<double, 3, 3>>(info_->k.data()).cast<float>().transpose();
    Sophus::SE3f T = camera_extrinsic_.value();
    optimizer_->setStaticParams(K, T);
    opt_pose = optimizer_->execute(cost_image, raw_pose, samples, &summary_text);
  }

  cv::Mat rgb_image;
  if (show_grad_image_) {
    cv::applyColorMap(cost_image, rgb_image, cv::COLORMAP_JET);
  } else {
    rgb_image = vml_common::decompress2CvMat(image_msg);
  }
  drawOverlayPoints(rgb_image, raw_pose, samples, cv::Scalar::all(0));
  drawOverlayLineSegments(rgb_image, opt_pose, line_segments, cv::Scalar(255, 255, 255));

  addText(summary_text, rgb_image);

  {
    String msg;
    msg.data = summary_text;
    pub_string_->publish(msg);
  }

  vml_common::publishImage(*pub_image_, rgb_image, stamp);

  PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";
  pose_stamped.header.stamp = stamp;
  pose_stamped.pose = vml_common::se32Pose(opt_pose);
  pub_pose_->publish(pose_stamped);
  {
    PoseCovStamped cov_msg;
    cov_msg.header = pose_stamped.header;
    cov_msg.pose.pose = pose_stamped.pose;
    cov_msg.pose.covariance.at(6 * 0 + 0) = 0.1;
    cov_msg.pose.covariance.at(6 * 1 + 1) = 0.1;
    cov_msg.pose.covariance.at(6 * 2 + 2) = 0.1;
    cov_msg.pose.covariance.at(6 * 3 + 3) = 0.01;
    cov_msg.pose.covariance.at(6 * 4 + 4) = 0.01;
    cov_msg.pose.covariance.at(6 * 5 + 5) = 0.01;
    pub_pose_cov_stamped_->publish(cov_msg);
  }
  // cv::imshow("6DoF fine optimization", rgb_image);
  // cv::waitKey(5);
}

void RefineOptimizer::drawOverlayLineSegments(
  cv::Mat & image, const Sophus::SE3f & pose_affine, const LineSegments & near_segments,
  const cv::Scalar & color)
{
  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3>>(info_->k.data()).cast<float>().transpose();
  Sophus::SE3f T = camera_extrinsic_.value();

  Sophus::SE3f transform = ground_plane_.alignWithSlope(pose_affine);

  auto project = [K, T, transform](const Eigen::Vector3f & xyz) -> std::optional<cv::Point2i> {
    Eigen::Vector3f from_camera = K * (T.inverse() * transform.inverse() * xyz);
    if (from_camera.z() < 1e-3f) return std::nullopt;
    Eigen::Vector3f uv1 = from_camera /= from_camera.z();
    return cv::Point2i(uv1.x(), uv1.y());
  };

  for (const pcl::PointNormal & pn : near_segments) {
    auto p1 = project(pn.getArray3fMap()), p2 = project(pn.getNormalVector3fMap());
    if (!p1.has_value() || !p2.has_value()) continue;
    cv::line(image, p1.value(), p2.value(), color, 3);
  }
}

void RefineOptimizer::drawOverlayPoints(
  cv::Mat & image, const Sophus::SE3f & pose_affine, const pcl::PointCloud<pcl::PointXYZ> & points,
  const cv::Scalar & color)
{
  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3>>(info_->k.data()).cast<float>().transpose();
  Sophus::SE3f T = camera_extrinsic_.value();

  Sophus::SE3f transform = ground_plane_.alignWithSlope(pose_affine);

  auto project = [K, T, transform](const Eigen::Vector3f & xyz) -> std::optional<cv::Point2i> {
    Eigen::Vector3f from_camera = K * (T.inverse() * transform.inverse() * xyz);
    if (from_camera.z() < 1e-3f) return std::nullopt;
    Eigen::Vector3f uv1 = from_camera /= from_camera.z();
    return cv::Point2i(uv1.x(), uv1.y());
  };

  for (const pcl::PointXYZ & p : points) {
    auto p1 = project(p.getArray3fMap());
    if (!p1.has_value()) continue;
    cv::circle(image, p1.value(), 3, color, -1);
  }
}

RefineOptimizer::LineSegments RefineOptimizer::extractNaerLineSegments(
  const Sophus::SE3f & pose, const LineSegments & line_segments)
{
  // Compute distance between linesegment and pose.
  // Note that the linesegment may pass close to the pose even if both end points are far from pose.
  auto checkIntersection = [this, pose](const pcl::PointNormal & pn) -> bool {
    const Eigen::Vector3f from = pose.inverse() * pn.getVector3fMap();
    const Eigen::Vector3f to = pose.inverse() * pn.getNormalVector3fMap();
    const Eigen::Vector3f tangent = to - from;

    if (tangent.squaredNorm() < 1e-3f) return false;

    // The closest point of the linesegment to pose
    float inner = from.dot(tangent);
    float mu = std::clamp(inner / tangent.squaredNorm(), 0.f, 1.0f);
    Eigen::Vector3f nearest = from + tangent * mu;

    if (nearest.x() < 0) return false;

    // The allowable distance along longitudinal direction is greater than one along
    // lateral direction. This is because line segments that are far apart along lateral
    // direction are not suitable for the overlaying optimization.
    float dx = nearest.x() / 60;  // allowable longitudinal error[m]
    float dy = nearest.y() / 10;  // allowable lateral error[m]
    return dx * dx + dy * dy < 1;
  };

  LineSegments near_linestrings;
  for (const pcl::PointNormal & pn : line_segments) {
    if (checkIntersection(pn)) {
      near_linestrings.push_back(pn);
    }
  }
  return near_linestrings;
}

cv::Mat RefineOptimizer::makeCostMap(LineSegments & line_segments)
{
  const cv::Size size(info_->width, info_->height);
  cv::Mat image = 255 * cv::Mat::ones(size, CV_8UC1);

  auto cvPoint = [](const Eigen::Vector3f & p) -> cv::Point2f { return cv::Point2f(p.x(), p.y()); };
  for (const auto pn : line_segments) {
    cv::Point2f from = cvPoint(pn.getVector3fMap());
    cv::Point2f to = cvPoint(pn.getNormalVector3fMap());
    cv::line(image, from, to, cv::Scalar::all(0), 1);
  }
  cv::Mat distance;
  cv::distanceTransform(image, distance, cv::DIST_L2, 3);
  cv::threshold(distance, distance, 100, 100, cv::THRESH_TRUNC);
  distance.convertTo(distance, CV_8UC1, -2.55, 255);

  return gamma_converter_(distance);
}

pcl::PointCloud<pcl::PointXYZ> RefineOptimizer::sampleUniformlyOnImage(
  const Sophus::SE3f & pose, const LineSegments & line_segments)
{
  Eigen::Matrix3f intrinsic =
    Eigen::Map<Eigen::Matrix<double, 3, 3>>(info_->k.data()).cast<float>().transpose();
  Sophus::SE3f extrinsic = camera_extrinsic_.value();

  auto pixelHash = [this](const Eigen::Vector2f & u) -> size_t {
    const int L = this->pixel_interval_;
    long x = static_cast<long>(std::floor(u.x() / L));
    long y = static_cast<long>(std::floor(u.y() / L));
    std::size_t seed = 0;
    boost::hash_combine(seed, x);
    boost::hash_combine(seed, y);
    return seed;
  };
  auto project = [pose, extrinsic,
                  intrinsic](const Eigen::Vector3f & p) -> std::optional<Eigen::Vector2f> {
    Eigen::Vector3f from_base_link = pose.inverse() * p;
    Eigen::Vector3f from_camera = extrinsic.inverse() * from_base_link;
    if (from_camera.z() < 1e-3f) return std::nullopt;
    Eigen::Vector3f u = intrinsic * from_camera / from_camera.z();
    return u.topRows(2);
  };

  std::unordered_set<size_t> already_occupied_pixels;
  pcl::PointCloud<pcl::PointXYZ> samples;
  for (const pcl::PointNormal & pn : line_segments) {
    const Eigen::Vector3f from = pn.getVector3fMap();
    const Eigen::Vector3f to = pn.getNormalVector3fMap();
    const Eigen::Vector3f t = (to - from).normalized();
    const float l = (from - to).norm();
    for (float distance = 0; distance < l; distance += 0.1f) {
      Eigen::Vector3f target = from + t * distance;
      std::optional<Eigen::Vector2f> u_opt = project(target);
      if ((!u_opt.has_value())) continue;

      size_t hash_id = pixelHash(u_opt.value());
      if (already_occupied_pixels.count(hash_id) != 0) continue;
      already_occupied_pixels.insert(hash_id);
      samples.push_back({target.x(), target.y(), target.z()});
    }
  }
  return samples;
}

}  // namespace refine_optimizer