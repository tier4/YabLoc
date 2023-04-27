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

#include "ll2_cost_map/direct_cost_map.hpp"

namespace yabloc
{
cv::Mat direct_cost_map(const cv::Mat & cost_map, const cv::Mat & intensity)
{
  constexpr int MAX_INT = std::numeric_limits<int>::max();

  std::vector<std::vector<int>> distances;
  distances.resize(cost_map.rows);
  for (int i = 0; i < cost_map.rows; i++) {
    distances.at(i).resize(cost_map.cols);
    std::fill(distances.at(i).begin(), distances.at(i).end(), MAX_INT);
    const uchar * intensity_ptr = intensity.ptr<uchar>(i);
    for (int j = 0; j < cost_map.cols; j++) {
      if (intensity_ptr[j] == 0) distances.at(i).at(j) = 0;
    }
  }

  cv::Mat dst = cost_map.clone();

  // Forward
  for (int r = 1; r < cost_map.rows; r++) {
    const uchar * upper_ptr = dst.ptr<uchar>(r - 1);
    uchar * current_ptr = dst.ptr<uchar>(r);

    for (int c = 1; c < cost_map.cols; c++) {
      int up = distances.at(r - 1).at(c);
      int left = distances.at(r).at(c - 1);
      if (up < left) {
        if (distances.at(r).at(c) < up + 1) continue;
        distances.at(r).at(c) = up + 1;
        current_ptr[c] = upper_ptr[c];
      } else {
        if (distances.at(r).at(c) < left + 1) continue;
        distances.at(r).at(c) = left + 1;
        current_ptr[c] = current_ptr[c - 1];
      }
    }
  }

  // Backward
  for (int r = cost_map.rows - 2; r >= 0; r--) {
    const uchar * downer_ptr = dst.ptr<uchar>(r + 1);
    uchar * current_ptr = dst.ptr<uchar>(r);

    for (int c = cost_map.cols - 2; c >= 0; c--) {
      int down = distances.at(r + 1).at(c);
      int right = distances.at(r).at(c + 1);
      if (down < right) {
        if (distances.at(r).at(c) < down + 1) continue;
        distances.at(r).at(c) = down + 1;
        current_ptr[c] = downer_ptr[c];
      } else {
        if (distances.at(r).at(c) < right + 1) continue;
        distances.at(r).at(c) = right + 1;
        current_ptr[c] = current_ptr[c + 1];
      }
    }
  }

  return dst;
}

cv::Mat visualize_direction_map(const cv::Mat & cost_map)
{
  cv::Mat s = 255 * cv::Mat::ones(cost_map.size(), CV_8UC1);
  cv::Mat v = 255 * cv::Mat::ones(cost_map.size(), CV_8UC1);
  cv::Mat hsv, rgb;
  cv::merge(std::vector<cv::Mat>{cost_map, s, v}, hsv);
  cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);

  for (int r = 0; r < cost_map.rows; r++) {
    const uchar * src_ptr = cost_map.ptr<uchar>(r);
    cv::Vec3b * dst_ptr = rgb.ptr<cv::Vec3b>(r);
    for (int c = 0; c < cost_map.cols; c++) {
      if (src_ptr[c] == 0) dst_ptr[c] = cv::Vec3b(0, 0, 0);
    }
  }
  return rgb;
}

}  // namespace yabloc

// #include <opencv4/opencv2/highgui.hpp>

// int main()
// {
//   // 0~180
//   cv::Mat raw_map = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
//   cv::line(raw_map, cv::Point(400, 0), cv::Point(400, 800), cv::Scalar::all(10), 3);
//   cv::line(raw_map, cv::Point(0, 400), cv::Point(800, 400), cv::Scalar::all(80), 3);
//   cv::line(raw_map, cv::Point(0, 0), cv::Point(400, 400), cv::Scalar::all(160), 3);
//   cv::line(raw_map, cv::Point(400, 400), cv::Point(800, 800), cv::Scalar::all(30), 3);

//   cv::Mat intensity = raw_map.clone();

//   cv::Mat directed = imgproc::directCostMap(raw_map, intensity);
//   cv::Mat show1 = imgproc::visualizeDirectionMap(raw_map);
//   cv::Mat show2 = imgproc::visualizeDirectionMap(directed);
//   cv::hconcat(show1, show2, show1);
//   cv::imshow("raw + directed", show1);
//   cv::waitKey(0);
// }