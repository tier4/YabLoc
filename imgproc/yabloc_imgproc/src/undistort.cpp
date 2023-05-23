#include "yabloc_imgproc/undistort.hpp"

#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <yabloc_common/cv_decompress.hpp>
#include <yabloc_common/timer.hpp>

namespace yabloc
{
void Undistort::make_remap_lut(const CameraInfo & info_msg)
{
  cv::Mat K = cv::Mat(cv::Size(3, 3), CV_64FC1, (void *)(info_msg.k.data()));
  cv::Mat D = cv::Mat(cv::Size(5, 1), CV_64FC1, (void *)(info_msg.d.data()));
  cv::Size size(info_msg.width, info_msg.height);

  cv::Size new_size = size;
  if (output_width_ > 0)
    new_size = cv::Size(output_width_, 1.0f * output_width_ / size.width * size.height);

  cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, D, size, 0, new_size);

  cv::initUndistortRectifyMap(
    K, D, cv::Mat(), new_K, new_size, CV_32FC1, undistort_map_x, undistort_map_y);

  scaled_info_ = sensor_msgs::msg::CameraInfo{};
  scaled_info_->k.at(0) = new_K.at<double>(0, 0);
  scaled_info_->k.at(2) = new_K.at<double>(0, 2);
  scaled_info_->k.at(4) = new_K.at<double>(1, 1);
  scaled_info_->k.at(5) = new_K.at<double>(1, 2);
  scaled_info_->k.at(8) = 1;
  scaled_info_->d.resize(5);
  scaled_info_->width = new_size.width;
  scaled_info_->height = new_size.height;
}

std::pair<cv::Mat, Undistort::CameraInfo> Undistort::undistort(
  const CompressedImage & image_msg, const CameraInfo & info_msg)
{
  if (undistort_map_x.empty()) make_remap_lut(info_msg);

  cv::Mat image = common::decompress_to_cv_mat(image_msg);

  cv::Mat undistorted_image;
  cv::remap(image, undistorted_image, undistort_map_x, undistort_map_y, cv::INTER_LINEAR);

  return {undistorted_image, scaled_info_.value()};
}

}  // namespace yabloc