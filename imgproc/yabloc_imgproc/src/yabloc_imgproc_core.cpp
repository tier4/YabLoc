#include "yabloc_imgproc/yabloc_imgproc.hpp"

#include <yabloc_common/pub_sub.hpp>
#include <yabloc_common/timer.hpp>

#include <cv_bridge/cv_bridge.h>

namespace yabloc
{

ImageProcessingNode::ImageProcessingNode()
: Node("image_processing"), override_frame_id_(declare_parameter<std::string>("override_frame_id"))
{
  using std::placeholders::_1;

  rclcpp::QoS qos{10};
  if (declare_parameter<bool>("use_sensor_qos")) {
    qos = rclcpp::QoS(10).durability_volatile().best_effort();
  }

  undistort_module_ = std::make_unique<Undistort>(this);
  lsd_module_ = std::make_unique<LineSegmentDetector>();
  graph_module_ = std::make_unique<GraphSegment>(this);
  filter_module_ = std::make_unique<SegmentFilter>(this);

  auto on_image = std::bind(&ImageProcessingNode::on_compressed_image, this, _1);
  auto on_info = std::bind(&ImageProcessingNode::on_info, this, _1);
  sub_image_ = create_subscription<CompressedImage>("src_image", qos, std::move(on_image));
  sub_info_ = create_subscription<CameraInfo>("src_info", qos, std::move(on_info));

  pub_info_ = create_publisher<CameraInfo>("resized_info", 10);
  pub_image_ = create_publisher<Image>("resized_image", 10);
  // line segments
  // pub_image_with_line_segments_ = create_publisher<Image>("image_with_line_segments", 10);
  // pub_cloud_ = create_publisher<PointCloud2>("line_segments_cloud", 10);
  // graph segmentation
  // pub_debug_image_ = create_publisher<Image>("segmented_image", 10);
  // segment filter
  pub_projected_cloud_ = create_publisher<PointCloud2>("projected_line_segments_cloud", 10);
  pub_debug_cloud_ = create_publisher<PointCloud2>("debug/line_segments_cloud", 10);
  pub_projected_image_ = create_publisher<Image>("projected_image", 10);

  filter_thread_ =
    std::make_shared<std::thread>(std::bind(&ImageProcessingNode::filter_thread_function, this));
  lsd_thread_ =
    std::make_shared<std::thread>(std::bind(&ImageProcessingNode::lsd_thread_function, this));
  graph_thread_ =
    std::make_shared<std::thread>(std::bind(&ImageProcessingNode::graph_thread_function, this));
}

void ImageProcessingNode::on_compressed_image(const CompressedImage image_msg)
{
  if (!info_.has_value()) return;
  const rclcpp::Time stamp = image_msg.header.stamp;

  common::Timer timer;

  // (1) undistort
  auto [scaled_image, scaled_info] = undistort_module_->undistort(image_msg, info_.value());
  {
    scaled_info.header = info_->header;
    if (override_frame_id_ != "") scaled_info.header.frame_id = override_frame_id_;
    publish_overriding_frame_id(image_msg, scaled_image, scaled_info);
  }

  // TODO: synchronization
  // Something bad happens when one node is abnormally late.
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    graph_scaled_image_queue_.push(scaled_image);
    graph_condition_.notify_one();
  }
  {
    std::lock_guard<std::mutex> lock(lsd_mutex_);
    lsd_scaled_image_queue_.push(scaled_image);
    lsd_condition_.notify_one();
  }
  {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    stamp_queue_.push(stamp);
    info_queue_.push(scaled_info);
    filter_condition_.notify_one();
  }

  RCLCPP_INFO_STREAM(get_logger(), "processing time-1: " << timer);
}

void ImageProcessingNode::graph_thread_function()
{
  while (rclcpp::ok()) {
    cv::Mat scaled_image;
    {
      std::unique_lock<std::mutex> lock(graph_mutex_);
      graph_condition_.wait(lock, [this] { return !graph_scaled_image_queue_.empty(); });
      scaled_image = graph_scaled_image_queue_.front();
      graph_scaled_image_queue_.pop();
    }

    common::Timer timer;
    // (3) grpah segmentation
    auto [segmented_image, colored_segmented_image] = graph_module_->execute(scaled_image);
    // common::publish_image(*pub_debug_image_, colored_segmented_image, image_msg.header.stamp);

    {
      std::lock_guard<std::mutex> lock(lsd_mutex_);
      graph_segment_queue_.push(segmented_image);
      filter_condition_.notify_one();
    }

    RCLCPP_INFO_STREAM(get_logger(), "processing time-3: " << timer);
  }
}

void ImageProcessingNode::lsd_thread_function()
{
  while (rclcpp::ok()) {
    cv::Mat scaled_image;
    {
      std::unique_lock<std::mutex> lock(lsd_mutex_);
      lsd_condition_.wait(lock, [this] { return !lsd_scaled_image_queue_.empty(); });
      scaled_image = lsd_scaled_image_queue_.front();
      lsd_scaled_image_queue_.pop();
    }

    common::Timer timer;
    // (2) line segment detection
    auto [line_segments_image, line_segments_cloud] = lsd_module_->execute(scaled_image);
    // common::publish_image(*pub_image_with_line_segments_, line_segments_image, stamp);
    // common::publish_cloud(*pub_cloud_, line_segments_cloud, stamp);

    {
      std::lock_guard<std::mutex> lock(lsd_mutex_);
      line_segments_queue_.push(line_segments_cloud);
      filter_condition_.notify_one();
    }
    RCLCPP_INFO_STREAM(get_logger(), "processing time-2: " << timer);
  }
}

void ImageProcessingNode::filter_thread_function()
{
  while (rclcpp::ok()) {
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointNormal> line_segments_cloud;
    cv::Mat segmented_image;
    CameraInfo scaled_info;

    {
      std::unique_lock<std::mutex> lock(filter_mutex_);
      filter_condition_.wait(
        lock, [this] { return !(line_segments_queue_.empty() || graph_segment_queue_.empty()); });

      stamp = stamp_queue_.front();
      line_segments_cloud = line_segments_queue_.front();
      segmented_image = graph_segment_queue_.front();
      scaled_info = info_queue_.front();
      stamp_queue_.pop();
      line_segments_queue_.pop();
      graph_segment_queue_.pop();
      info_queue_.pop();
    }
    common::Timer timer;

    // (4) segment filter
    pcl::PointCloud<pcl::PointXYZLNormal> combined_edges, combined_debug_edges;
    cv::Mat projected_image;
    try {
      filter_module_->set_info(scaled_info);
      std::tie(combined_edges, combined_debug_edges, projected_image) =
        filter_module_->execute(line_segments_cloud, segmented_image);
    } catch (...) {
      RCLCPP_ERROR_STREAM(get_logger(), "catched error");
      return;
    }
    common::publish_cloud(*pub_projected_cloud_, combined_edges, stamp);
    common::publish_cloud(*pub_debug_cloud_, combined_debug_edges, stamp);
    common::publish_image(*pub_projected_image_, projected_image, stamp);
    RCLCPP_INFO_STREAM(get_logger(), "processing time-4: " << timer);
  }
}

void ImageProcessingNode::publish_overriding_frame_id(
  const CompressedImage & image_msg, const cv::Mat & scaled_image, const CameraInfo & scaled_info)
{
  // Publish CameraInfo
  {
    pub_info_->publish(scaled_info);
  }

  // Publish Image
  {
    cv_bridge::CvImage bridge;
    bridge.header.stamp = image_msg.header.stamp;
    if (override_frame_id_ != "")
      bridge.header.frame_id = override_frame_id_;
    else
      bridge.header.frame_id = image_msg.header.frame_id;
    bridge.encoding = "bgr8";
    bridge.image = scaled_image;
    pub_image_->publish(*bridge.toImageMsg());
  }
}

}  // namespace yabloc