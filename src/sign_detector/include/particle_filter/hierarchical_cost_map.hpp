#pragma once
#include "common/gammma_conveter.hpp"

#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/core.hpp>

#include <boost/functional/hash.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace particle_filter
{
struct Area
{
  Area() {}
  Area(const Eigen::Vector2f & v)
  {
    if (unit_length_ < 0) throwError();
    x = static_cast<long>(std::floor(v.x() / unit_length_));
    y = static_cast<long>(std::floor(v.y() / unit_length_));
  }
  void throwError() const
  {
    std::cerr << "Are::unit_length_ is not initialized" << std::endl;
    exit(EXIT_FAILURE);
  }

  int x, y;
  static float unit_length_;

  friend bool operator==(const Area & one, const Area & other)
  {
    return one.x == other.x && one.y == other.y;
  }
  friend bool operator!=(const Area & one, const Area & other) { return !(one == other); }
  size_t operator()(const Area & index) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, index.x);
    boost::hash_combine(seed, index.y);
    return seed;
  }

  static cv::Point toCvPoint(const Eigen::Vector3f);
};

class HierarchicalCostMap
{
public:
  HierarchicalCostMap(float max_range, float image_size, float gamma);

  void setCloud(const pcl::PointCloud<pcl::PointNormal> & cloud);

  float at(const Eigen::Vector2f & position);

private:
  const float max_range_;
  const float image_size_;

  void buildMap(const Area & area);
  GammaConverter gamma_converter{4.0f};

  std::optional<pcl::PointCloud<pcl::PointNormal>> cloud_;
  std::unordered_map<Area, cv::Mat, Area> cost_maps;
};
}  // namespace particle_filter