// aggregator_core.hpp  -- V1: mirrors current PointCloudAggregator behaviour
// Pure C++17, no ROS2/PCL dependency -- used for testing and benchmarking.
#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <unordered_set>
#include <vector>

namespace lidar_processor_cpp
{

struct AggregatorConfig
{
  int max_points;
  explicit AggregatorConfig(int mp = 1000000) : max_points(mp) {}
};

struct Point3D
{
  float x, y, z;
  Point3D(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}
  bool operator==(const Point3D & other) const
  {
    return std::abs(x - other.x) < 1e-6f &&
           std::abs(y - other.y) < 1e-6f &&
           std::abs(z - other.z) < 1e-6f;
  }
};

// V1 hash: has a bit-overlap collision bug when |y| >= 65.535 m or |z| >= 0.065 m
// at 1 mm resolution (y_int >= 65536 shifts 17+ bits, overlaps x_int region).
struct Point3DHashV1
{
  std::size_t operator()(const Point3D & p) const
  {
    int x_int = static_cast<int>(std::round(p.x * 1000));
    int y_int = static_cast<int>(std::round(p.y * 1000));
    int z_int = static_cast<int>(std::round(p.z * 1000));
    return std::hash<long long>{}(
      (static_cast<long long>(x_int) << 32) |
      (static_cast<long long>(y_int) << 16) |
      static_cast<long long>(z_int));
  }
};

// V1 aggregator: exact copy of current PointCloudAggregator logic.
class PointCloudAggregatorV1
{
public:
  explicit PointCloudAggregatorV1(const AggregatorConfig & config)
  : config_(config), points_changed_(false)
  {
  }

  void addPoints(const std::vector<Point3D> & new_points)
  {
    std::lock_guard<std::mutex> lock(points_mutex_);

    for (const auto & point : new_points) {
      Point3D rounded(
        std::round(point.x * 1000.0f) / 1000.0f,
        std::round(point.y * 1000.0f) / 1000.0f,
        std::round(point.z * 1000.0f) / 1000.0f);
      points_.insert(rounded);
    }

    // Overflow: O(N log N) -- copy + sort + re-insert
    if (static_cast<int>(points_.size()) > config_.max_points) {
      std::vector<Point3D> pv(points_.begin(), points_.end());
      std::sort(pv.begin(), pv.end(), [](const Point3D & a, const Point3D & b) {
          float da = a.x * a.x + a.y * a.y + a.z * a.z;
          float db = b.x * b.x + b.y * b.y + b.z * b.z;
          return da < db;
        });
      points_.clear();
      for (int i = 0; i < config_.max_points && i < static_cast<int>(pv.size()); ++i) {
        points_.insert(pv[i]);
      }
    }

    points_changed_ = true;
  }

  std::vector<Point3D> getPointsCopy() const
  {
    std::lock_guard<std::mutex> lock(points_mutex_);
    return std::vector<Point3D>(points_.begin(), points_.end());
  }

  int getPointCount() const
  {
    std::lock_guard<std::mutex> lock(points_mutex_);
    return static_cast<int>(points_.size());
  }

private:
  AggregatorConfig config_;
  std::unordered_set<Point3D, Point3DHashV1> points_;
  mutable std::mutex points_mutex_;
  std::atomic<bool> points_changed_;
};

}  // namespace lidar_processor_cpp
