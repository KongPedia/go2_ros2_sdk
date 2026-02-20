// lidar_to_pointcloud_node_v2.hpp -- V2: uses voxel hash aggregator
// Drop-in replacement for lidar_to_pointcloud_node.hpp.
// Kept in a separate file for A/B comparison; original is unchanged.
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/ply_io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

#include "lidar_processor_cpp/aggregator_core_v2.hpp"

namespace lidar_processor_cpp
{

struct LidarConfigV2
{
  std::vector<std::string> robot_ip_list;
  std::string map_name;
  bool save_map;
  double save_interval;
  int max_points;
  double voxel_size;
};

class LidarToPointCloudNodeV2 : public rclcpp::Node
{
public:
  LidarToPointCloudNodeV2();

private:
  void declareParameters();
  LidarConfigV2 loadConfiguration();
  void setupSubscriptions();
  void setupPublishers();
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishAggregatedPointcloud(const std_msgs::msg::Header & header);
  void saveMapCallback();
  void logConfiguration();

  LidarConfigV2 config_;
  std::unique_ptr<PointCloudAggregatorV2> aggregator_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::TimerBase::SharedPtr save_timer_;
};

}  // namespace lidar_processor_cpp
