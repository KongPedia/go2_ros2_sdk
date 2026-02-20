// lidar_to_pointcloud_node_v2.cpp -- V2: voxel hash aggregator
// Kept as a separate file alongside the original for A/B comparison.
// Key differences vs V1:
//   - PointCloudAggregatorV2: uint64_t voxel key, no O(N log N) sort on overflow
//   - Hash function: non-overlapping 21-bit packing, no collision at y>=65 m

#include "lidar_processor_cpp/lidar_to_pointcloud_node_v2.hpp"
#include <algorithm>
#include <chrono>

namespace lidar_processor_cpp
{

LidarToPointCloudNodeV2::LidarToPointCloudNodeV2()
: Node("lidar_to_pointcloud_v2")
{
  declareParameters();
  config_ = loadConfiguration();

  AggregatorConfigV2 agg_cfg(
    config_.max_points,
    static_cast<float>(config_.voxel_size));
  aggregator_ = std::make_unique<PointCloudAggregatorV2>(agg_cfg);

  setupSubscriptions();
  setupPublishers();

  if (config_.save_map) {
    save_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(config_.save_interval),
      std::bind(&LidarToPointCloudNodeV2::saveMapCallback, this));
  }

  logConfiguration();
}

void LidarToPointCloudNodeV2::declareParameters()
{
  this->declare_parameter("robot_ip_lst", std::vector<std::string>{});
  this->declare_parameter("map_name", "3d_map");
  this->declare_parameter("map_save", "true");
  this->declare_parameter("save_interval", 10.0);
  this->declare_parameter("max_points", 1000000);
  this->declare_parameter("voxel_size", 0.001);
}

LidarConfigV2 LidarToPointCloudNodeV2::loadConfiguration()
{
  LidarConfigV2 config;
  config.robot_ip_list = this->get_parameter("robot_ip_lst").as_string_array();
  config.map_name = this->get_parameter("map_name").as_string();
  config.save_map = (this->get_parameter("map_save").as_string() == "true");
  config.save_interval = this->get_parameter("save_interval").as_double();
  config.max_points = this->get_parameter("max_points").as_int();
  config.voxel_size = this->get_parameter("voxel_size").as_double();
  return config;
}

void LidarToPointCloudNodeV2::setupSubscriptions()
{
  auto qos = rclcpp::QoS(1)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);

  if (config_.robot_ip_list.size() == 1) {
    subscriptions_.push_back(
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/robot0/point_cloud2", qos,
        std::bind(&LidarToPointCloudNodeV2::lidarCallback, this,
          std::placeholders::_1)));
  } else {
    for (size_t i = 0; i < config_.robot_ip_list.size(); ++i) {
      subscriptions_.push_back(
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/robot" + std::to_string(i) + "/point_cloud2", qos,
          std::bind(&LidarToPointCloudNodeV2::lidarCallback, this,
            std::placeholders::_1)));
    }
  }
}

void LidarToPointCloudNodeV2::setupPublishers()
{
  auto qos = rclcpp::QoS(1)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);

  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/aggregated", qos);
}

void LidarToPointCloudNodeV2::lidarCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  try {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    std::vector<Point3DV2> points;
    points.reserve(cloud->points.size());
    for (const auto & p : cloud->points) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        points.emplace_back(p.x, p.y, p.z);
      }
    }

    aggregator_->addPoints(points);
    publishAggregatedPointcloud(msg->header);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing LiDAR data: %s", e.what());
  }
}

void LidarToPointCloudNodeV2::publishAggregatedPointcloud(
  const std_msgs::msg::Header & header)
{
  try {
    auto points = aggregator_->getPointsCopy();
    if (points.empty()) {return;}

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(points.size());
    for (const auto & p : points) {
      cloud->points.emplace_back(p.x, p.y, p.z);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = header;
    pointcloud_pub_->publish(msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing point cloud: %s", e.what());
  }
}

void LidarToPointCloudNodeV2::saveMapCallback()
{
  try {
    auto points = aggregator_->getPointsCopy();
    if (points.empty()) {return;}

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(points.size());
    for (const auto & p : points) {
      cloud->points.emplace_back(p.x, p.y, p.z);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    if (config_.voxel_size > 0) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(
        static_cast<float>(config_.voxel_size),
        static_cast<float>(config_.voxel_size),
        static_cast<float>(config_.voxel_size));
      vg.filter(*ds);
      cloud = ds;
    }

    const std::string fname = config_.map_name + "_v2.ply";
    if (pcl::io::savePLYFileBinary(fname, *cloud) == 0) {
      RCLCPP_INFO(this->get_logger(), "Saved map: %s (%zu points)",
        fname.c_str(), cloud->points.size());
      aggregator_->clear();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", fname.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error saving map: %s", e.what());
  }
}

void LidarToPointCloudNodeV2::logConfiguration()
{
  RCLCPP_INFO(this->get_logger(), "LiDAR Processor V2 (voxel hash aggregator):");
  RCLCPP_INFO(this->get_logger(), "  max_points  : %d", config_.max_points);
  RCLCPP_INFO(this->get_logger(), "  voxel_size  : %.4f m", config_.voxel_size);
  RCLCPP_INFO(this->get_logger(), "  save_map    : %s", config_.save_map ? "true" : "false");
}

}  // namespace lidar_processor_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<lidar_processor_cpp::LidarToPointCloudNodeV2>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
