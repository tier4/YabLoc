#include "common/util.hpp"
#include "validation/overlay.hpp"
#include "validation/refine.hpp"

#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <sophus/geometry.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace validation
{
RefineOptimizer::RefineOptimizer() : Node("refine"), pose_buffer_{40}, tf_subscriber_(get_clock())
{
  using std::placeholders::_1, std::placeholders::_2;

  auto cb_synchro = std::bind(&RefineOptimizer::imageAndLsdCallback, this, _1, _2);
  sub_synchro_ =
    std::make_shared<SynchroSubscriber<Image, PointCloud2>>(this, "/src_image", "/lsd_cloud");
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

  opt_config_ = RefineConfig{this};
  gamma_converter_.reset(5.0);
}

void RefineOptimizer::infoCallback(const CameraInfo & msg)
{
  info_ = msg;
  camera_extrinsic_ = tf_subscriber_.se3f(info_->header.frame_id, "base_link");
}

void addText(const std::string & text, const cv::Mat & image)
{
  const int fontscale = 2;
  const int fontface = cv::FONT_HERSHEY_PLAIN;
  const int thickness = 2;
  int cumulative_height = 10;
  int max_width = 10;

  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    int balse_line;
    cv::Size size = cv::getTextSize(line, fontface, fontscale, thickness, &balse_line);
    cumulative_height += (size.height + 5);
    max_width = std::max(max_width, size.width);
  }
  cv::rectangle(
    image, cv::Rect(40 + 5, 20 - 5, max_width, cumulative_height), cv::Scalar::all(0), -1);
  cumulative_height = 10;

  ss = std::stringstream(text);
  while (std::getline(ss, line)) {
    int balse_line;
    cv::Size size = cv::getTextSize(line, fontface, fontscale, thickness, &balse_line);
    cumulative_height += (size.height + 5);
    cv::putText(
      image, line, cv::Point2i(50, cumulative_height), fontface, fontscale, cv::Scalar::all(255),
      thickness);
  }
}

void RefineOptimizer::imageAndLsdCallback(const Image & image_msg, const PointCloud2 & lsd_msg)
{
  const rclcpp::Time stamp = lsd_msg.header.stamp;

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

  LineSegments lsd;
  pcl::fromROSMsg(lsd_msg, lsd);
  cv::Mat cost_image = makeCostMap(lsd);

  Sophus::SE3f raw_pose = util::pose2Se3(synched_pose.pose);
  auto linesegments = extractNaerLineSegments(raw_pose, ll2_cloud_);

  Sophus::SE3f opt_pose;
  std::string summary_text;
  {
    // Optimization
    Eigen::Matrix3f K =
      Eigen::Map<Eigen::Matrix<double, 3, 3>>(info_->k.data()).cast<float>().transpose();
    Sophus::SE3f T = camera_extrinsic_.value();

    opt_pose = refinePose(T, K, cost_image, raw_pose, linesegments, opt_config_, &summary_text);
  }

  cv::Mat rgb_image;
  cv::applyColorMap(cost_image, rgb_image, cv::COLORMAP_JET);
  // cv::Mat rgb_image = util::decompress2CvMat(image_msg);
  drawOverlayLineSegments(rgb_image, raw_pose, linesegments, cv::Scalar(0, 0, 0));
  drawOverlayLineSegments(rgb_image, opt_pose, linesegments, cv::Scalar(255, 255, 255));

  addText(summary_text, rgb_image);

  cv::imshow("6DoF fine optimization", rgb_image);
  cv::waitKey(5);
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

RefineOptimizer::LineSegments RefineOptimizer::extractNaerLineSegments(
  const Sophus::SE3f & pose, const LineSegments & linesegments)
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
  for (const pcl::PointNormal & pn : linesegments) {
    if (checkIntersection(pn)) {
      near_linestrings.push_back(pn);
    }
  }
  return near_linestrings;
}

cv::Mat RefineOptimizer::makeCostMap(LineSegments & lsd)
{
  const cv::Size size(info_->width, info_->height);
  cv::Mat image = 255 * cv::Mat::ones(size, CV_8UC1);

  auto cvPoint = [](const Eigen::Vector3f & p) -> cv::Point2f { return cv::Point2f(p.x(), p.y()); };
  for (const auto pn : lsd) {
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

}  // namespace validation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<validation::RefineOptimizer>());
  rclcpp::shutdown();
  return 0;
}