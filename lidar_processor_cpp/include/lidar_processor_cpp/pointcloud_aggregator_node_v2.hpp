// pointcloud_aggregator_node_v2.hpp -- V2: SOR removed, range+height filter only.
// Kept alongside original for A/B comparison; original file is unchanged.
//
// Rationale: StatisticalOutlierRemoval (SOR) builds a KD-tree for every
// callback and queries k=20 nearest-neighbours per point.  At 120k points this
// is O(N*k*log N) ≈ 500 ms per frame on x86 (and 3-10x slower on Jetson ARM),
// making 10 Hz operation impossible.  Range + height filtering is O(N) and
// takes ~1.5 ms for the same data.
#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

namespace lidar_processor_cpp
{

struct AggregatorConfigV2
{
  double max_range;
  double min_range;
  double height_filter_min;
  double height_filter_max;
  int downsample_rate;
  double publish_rate;
};

class PointCloudAggregatorNodeV2 : public rclcpp::Node
{
public:
  PointCloudAggregatorNodeV2();

private:
  void declareParameters();
  AggregatorConfigV2 loadConfiguration();
  void setupSubscriptions();
  void setupPublishers();
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr applyFilters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud);
  void publishCallback();
  void logConfiguration();

  AggregatorConfigV2 config_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> aggregated_clouds_;
  std::chrono::steady_clock::time_point last_publish_time_;
  mutable std::mutex clouds_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace lidar_processor_cpp
