// pointcloud_aggregator_node_v2.cpp -- V2: SOR removed, range+height filter only.
// All other behaviour is identical to V1; this file is a drop-in replacement.

#include "lidar_processor_cpp/pointcloud_aggregator_node_v2.hpp"
#include <algorithm>
#include <cmath>

namespace lidar_processor_cpp
{

PointCloudAggregatorNodeV2::PointCloudAggregatorNodeV2()
: Node("pointcloud_aggregator_v2")
{
  declareParameters();
  config_ = loadConfiguration();
  last_publish_time_ = std::chrono::steady_clock::now();

  setupSubscriptions();
  setupPublishers();

  publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / config_.publish_rate),
    std::bind(&PointCloudAggregatorNodeV2::publishCallback, this));

  RCLCPP_INFO(this->get_logger(), "PointCloud Aggregator V2 initialized (SOR removed)");
  logConfiguration();
}

void PointCloudAggregatorNodeV2::declareParameters()
{
  this->declare_parameter("max_range", 20.0);
  this->declare_parameter("min_range", 0.1);
  this->declare_parameter("height_filter_min", -2.0);
  this->declare_parameter("height_filter_max", 3.0);
  this->declare_parameter("downsample_rate", 10);
  this->declare_parameter("publish_rate", 5.0);
}

AggregatorConfigV2 PointCloudAggregatorNodeV2::loadConfiguration()
{
  AggregatorConfigV2 cfg;
  cfg.max_range = this->get_parameter("max_range").as_double();
  cfg.min_range = this->get_parameter("min_range").as_double();
  cfg.height_filter_min = this->get_parameter("height_filter_min").as_double();
  cfg.height_filter_max = this->get_parameter("height_filter_max").as_double();
  cfg.downsample_rate = this->get_parameter("downsample_rate").as_int();
  cfg.publish_rate = this->get_parameter("publish_rate").as_double();
  return cfg;
}

void PointCloudAggregatorNodeV2::setupSubscriptions()
{
  auto qos = rclcpp::QoS(5)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/aggregated", qos,
    std::bind(&PointCloudAggregatorNodeV2::pointcloudCallback, this,
      std::placeholders::_1));
}

void PointCloudAggregatorNodeV2::setupPublishers()
{
  auto qos = rclcpp::QoS(5)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);

  filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/filtered", qos);
  downsampled_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/downsampled", qos);
}

void PointCloudAggregatorNodeV2::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  try {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->points.empty()) {return;}

    auto filtered = applyFilters(cloud);
    if (!filtered->points.empty()) {
      std::lock_guard<std::mutex> lock(clouds_mutex_);
      aggregated_clouds_.push_back(filtered);

      const size_t max_clouds = static_cast<size_t>(config_.publish_rate * 10);
      if (aggregated_clouds_.size() > max_clouds) {
        aggregated_clouds_.erase(
          aggregated_clouds_.begin(),
          aggregated_clouds_.begin() +
            static_cast<std::ptrdiff_t>(aggregated_clouds_.size() - max_clouds));
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in pointcloud callback: %s", e.what());
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudAggregatorNodeV2::applyFilters(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud)
{
  // V2: range + height filter ONLY -- no StatisticalOutlierRemoval.
  // SOR was O(N*k*log N); this is O(N).
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  out->points.reserve(input_cloud->points.size());

  const float min_r2 = static_cast<float>(config_.min_range * config_.min_range);
  const float max_r2 = static_cast<float>(config_.max_range * config_.max_range);
  const float z_min = static_cast<float>(config_.height_filter_min);
  const float z_max = static_cast<float>(config_.height_filter_max);

  for (const auto & p : input_cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}

    const float r2 = p.x * p.x + p.y * p.y;
    if (r2 < min_r2 || r2 > max_r2) {continue;}
    if (p.z < z_min || p.z > z_max) {continue;}

    out->points.push_back(p);
  }

  out->width = out->points.size();
  out->height = 1;
  out->is_dense = true;
  return out;
}

void PointCloudAggregatorNodeV2::publishCallback()
{
  try {
    std::lock_guard<std::mutex> lock(clouds_mutex_);
    if (aggregated_clouds_.empty()) {return;}

    pcl::PointCloud<pcl::PointXYZ>::Ptr combined(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & c : aggregated_clouds_) {
      *combined += *c;
    }
    if (combined->points.empty()) {return;}

    combined->width = combined->points.size();
    combined->height = 1;
    combined->is_dense = true;

    std_msgs::msg::Header hdr;
    hdr.stamp = this->get_clock()->now();
    hdr.frame_id = "base_link";

    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*combined, filtered_msg);
    filtered_msg.header = hdr;
    filtered_pub_->publish(filtered_msg);

    if (config_.downsample_rate > 1) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
      const int rate = config_.downsample_rate;
      for (size_t i = 0; i < combined->points.size(); i += static_cast<size_t>(rate)) {
        ds->points.push_back(combined->points[i]);
      }
      ds->width = ds->points.size();
      ds->height = 1;
      ds->is_dense = true;

      sensor_msgs::msg::PointCloud2 ds_msg;
      pcl::toROSMsg(*ds, ds_msg);
      ds_msg.header = hdr;
      downsampled_pub_->publish(ds_msg);
    }

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_publish_time_).count() >= 5) {
      RCLCPP_INFO(this->get_logger(), "Published %zu points (%zu clouds)",
        combined->points.size(), aggregated_clouds_.size());
      last_publish_time_ = now;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in publish callback: %s", e.what());
  }
}

void PointCloudAggregatorNodeV2::logConfiguration()
{
  RCLCPP_INFO(this->get_logger(), "  Range   : [%.2f, %.2f] m",
    config_.min_range, config_.max_range);
  RCLCPP_INFO(this->get_logger(), "  Height  : [%.2f, %.2f] m",
    config_.height_filter_min, config_.height_filter_max);
  RCLCPP_INFO(this->get_logger(), "  Rate    : %.1f Hz, downsample: 1/%d",
    config_.publish_rate, config_.downsample_rate);
}

}  // namespace lidar_processor_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<lidar_processor_cpp::PointCloudAggregatorNodeV2>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
