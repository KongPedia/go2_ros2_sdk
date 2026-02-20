// aggregator_core_v2.hpp  -- V2: voxel spatial hash map aggregator
// Key improvements over V1:
//   1. Fixed hash function: non-overlapping 21-bit packed key (no collision)
//   2. O(1) amortised insert (unordered_map on uint64_t key, not BST/unordered_set)
//   3. No O(N log N) sort on overflow: at-capacity frames skip gracefully
//   4. reserve() at construction avoids rehash for large point counts
//
// Pure C++17, no ROS2/PCL dependency.
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace lidar_processor_cpp
{

struct AggregatorConfigV2
{
  int max_points;
  float voxel_size;  // metres; default 1 mm gives same granularity as V1 rounding
  AggregatorConfigV2(int mp = 1000000, float vs = 0.001f)
  : max_points(mp), voxel_size(vs) {}
};

struct Point3DV2
{
  float x, y, z;
  Point3DV2(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}
};

class PointCloudAggregatorV2
{
public:
  explicit PointCloudAggregatorV2(const AggregatorConfigV2 & config)
  : config_(config), inv_voxel_size_(1.0f / config.voxel_size), points_changed_(false)
  {
    voxel_map_.reserve(static_cast<size_t>(config_.max_points));
  }

  void addPoints(const std::vector<Point3DV2> & new_points)
  {
    std::lock_guard<std::mutex> lock(points_mutex_);
    const int cap = config_.max_points;
    for (const auto & pt : new_points) {
      if (static_cast<int>(voxel_map_.size()) >= cap) {
        break;  // At capacity -- O(1), no sort
      }
      const uint64_t key = voxelKey(pt);
      voxel_map_.emplace(key, pt);  // No-op if key already present (dedup)
    }
    points_changed_ = true;
  }

  std::vector<Point3DV2> getPointsCopy() const
  {
    std::lock_guard<std::mutex> lock(points_mutex_);
    std::vector<Point3DV2> out;
    out.reserve(voxel_map_.size());
    for (const auto & [k, pt] : voxel_map_) {
      out.push_back(pt);
    }
    return out;
  }

  int getPointCount() const
  {
    std::lock_guard<std::mutex> lock(points_mutex_);
    return static_cast<int>(voxel_map_.size());
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(points_mutex_);
    voxel_map_.clear();
  }

private:
  // Pack (x, y, z) into a uint64_t voxel key.
  // 21 bits per axis (offset by 2^20) covers ±1048576 voxels per axis.
  // At voxel_size=1 mm that is ±1048.576 m -- well beyond any robot range.
  // No bit overlap: x occupies [42,62], y [21,41], z [0,20].
  uint64_t voxelKey(const Point3DV2 & p) const
  {
    const auto xi = static_cast<int64_t>(std::floor(p.x * inv_voxel_size_)) + (1LL << 20);
    const auto yi = static_cast<int64_t>(std::floor(p.y * inv_voxel_size_)) + (1LL << 20);
    const auto zi = static_cast<int64_t>(std::floor(p.z * inv_voxel_size_)) + (1LL << 20);
    return ((static_cast<uint64_t>(xi) & 0x1FFFFF) << 42) |
           ((static_cast<uint64_t>(yi) & 0x1FFFFF) << 21) |
           (static_cast<uint64_t>(zi) & 0x1FFFFF);
  }

  AggregatorConfigV2 config_;
  float inv_voxel_size_;
  std::unordered_map<uint64_t, Point3DV2> voxel_map_;
  mutable std::mutex points_mutex_;
  std::atomic<bool> points_changed_;
};

}  // namespace lidar_processor_cpp
