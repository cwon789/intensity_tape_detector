#include "tape_detector_ros2/tape_detector_node.hpp"
#include <chrono>

namespace tape_detector_ros2
{

TapeDetectorNode::TapeDetectorNode(const rclcpp::NodeOptions & options)
  : Node("tape_detector_node", options)
{
  this->declare_parameter("min_range", 0.1);
  this->declare_parameter("max_range", 30.0);
  this->declare_parameter("intensity_threshold", 200.0);
  this->declare_parameter("cluster_tolerance", 0.1);
  this->declare_parameter("min_cluster_size", 3);
  this->declare_parameter("search_radius", 0.5);
  
  min_range_ = this->get_parameter("min_range").as_double();
  max_range_ = this->get_parameter("max_range").as_double();
  intensity_threshold_ = this->get_parameter("intensity_threshold").as_double();
  cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
  min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
  search_radius_ = this->get_parameter("search_radius").as_double();
  
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&TapeDetectorNode::laserScanCallback, this, std::placeholders::_1));
  
  hi_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/high_intensity_points", 10);
  
  li_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/stable_feature_points", 10);
  
  RCLCPP_INFO(this->get_logger(), 
    "Tape Detector Node initialized with parameters:\n"
    "  - min_range: %.2f\n"
    "  - max_range: %.2f\n"
    "  - intensity_threshold: %.1f\n"
    "  - cluster_tolerance: %.3f\n"
    "  - min_cluster_size: %d\n"
    "  - search_radius: %.2f",
    min_range_, max_range_, intensity_threshold_, 
    cluster_tolerance_, min_cluster_size_, search_radius_);
}

void TapeDetectorNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  
  std::vector<Point2D> filtered_points = preprocessAndFilter(msg);
  
  if (filtered_points.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "No valid points after filtering");
    return;
  }
  
  std::vector<Cluster> hi_clusters = clusterHighIntensityPoints(filtered_points);
  
  std::vector<Point2D> li_features;
  for (auto& cluster : hi_clusters) {
    cluster.calculateCentroid();
    
    Point2D li_feature = findLowIntensityFeature(cluster.centroid, filtered_points);
    if (li_feature.x != 0.0 || li_feature.y != 0.0) {
      li_features.push_back(li_feature);
    }
  }
  
  publishPointClouds(hi_clusters, li_features);
  
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  
  RCLCPP_DEBUG(this->get_logger(), 
    "Processing completed in %ld us. Found %zu HI clusters and %zu LI features",
    duration.count(), hi_clusters.size(), li_features.size());
}

std::vector<Point2D> TapeDetectorNode::preprocessAndFilter(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::vector<Point2D> filtered_points;
  
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float range = msg->ranges[i];
    
    if (std::isfinite(range) && range >= min_range_ && range <= max_range_) {
      float angle = msg->angle_min + i * msg->angle_increment;
      
      double x = range * std::cos(angle);
      double y = range * std::sin(angle);
      
      float intensity = 0.0;
      if (!msg->intensities.empty() && i < msg->intensities.size()) {
        intensity = msg->intensities[i];
      }
      
      filtered_points.emplace_back(x, y, intensity);
    }
  }
  
  return filtered_points;
}

std::vector<Cluster> TapeDetectorNode::clusterHighIntensityPoints(
  const std::vector<Point2D>& points)
{
  std::vector<Point2D> hi_points;
  for (const auto& p : points) {
    if (p.intensity > intensity_threshold_) {
      hi_points.push_back(p);
    }
  }
  
  if (hi_points.empty()) {
    return {};
  }
  
  std::vector<Cluster> clusters;
  performEuclideanClustering(hi_points, clusters, cluster_tolerance_);
  
  clusters.erase(
    std::remove_if(clusters.begin(), clusters.end(),
      [this](const Cluster& c) { 
        return static_cast<int>(c.points.size()) < min_cluster_size_; 
      }),
    clusters.end());
  
  return clusters;
}

void TapeDetectorNode::performEuclideanClustering(
  const std::vector<Point2D>& points,
  std::vector<Cluster>& clusters,
  double tolerance)
{
  std::vector<bool> processed(points.size(), false);
  
  for (size_t i = 0; i < points.size(); ++i) {
    if (processed[i]) continue;
    
    Cluster new_cluster;
    std::vector<size_t> seed_queue;
    seed_queue.push_back(i);
    processed[i] = true;
    
    size_t current_seed = 0;
    while (current_seed < seed_queue.size()) {
      size_t seed_idx = seed_queue[current_seed];
      new_cluster.points.push_back(points[seed_idx]);
      
      for (size_t j = 0; j < points.size(); ++j) {
        if (!processed[j]) {
          double dist = euclideanDistance(points[seed_idx], points[j]);
          if (dist <= tolerance) {
            seed_queue.push_back(j);
            processed[j] = true;
          }
        }
      }
      current_seed++;
    }
    
    clusters.push_back(new_cluster);
  }
}

Point2D TapeDetectorNode::findLowIntensityFeature(
  const Point2D& hi_centroid,
  const std::vector<Point2D>& all_points)
{
  std::vector<Point2D> nearby_li_points;
  
  for (const auto& p : all_points) {
    if (p.intensity < intensity_threshold_) {
      double dist = euclideanDistance(hi_centroid, p);
      if (dist < search_radius_) {
        nearby_li_points.push_back(p);
      }
    }
  }
  
  if (nearby_li_points.empty()) {
    return Point2D(0.0, 0.0, 0.0);
  }
  
  double sum_x = 0.0, sum_y = 0.0;
  for (const auto& p : nearby_li_points) {
    sum_x += p.x;
    sum_y += p.y;
  }
  
  return Point2D(
    sum_x / nearby_li_points.size(),
    sum_y / nearby_li_points.size(),
    0.0);
}

void TapeDetectorNode::publishPointClouds(
  const std::vector<Cluster>& hi_clusters,
  const std::vector<Point2D>& li_features)
{
  std::vector<Point2D> hi_centroids;
  for (const auto& cluster : hi_clusters) {
    hi_centroids.push_back(cluster.centroid);
  }
  
  if (!hi_centroids.empty()) {
    auto hi_cloud = createPointCloud(hi_centroids, "lidar_front");
    hi_cloud_pub_->publish(hi_cloud);
  }
  
  if (!li_features.empty()) {
    auto li_cloud = createPointCloud(li_features, "lidar_front");
    li_cloud_pub_->publish(li_cloud);
  }
}

sensor_msgs::msg::PointCloud2 TapeDetectorNode::createPointCloud(
  const std::vector<Point2D>& points,
  const std::string& frame_id)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = this->now();
  cloud.height = 1;
  cloud.width = points.size();
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  
  modifier.resize(points.size());
  
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
  
  for (const auto& point : points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = 0.0f;
    *iter_intensity = point.intensity;
    
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  
  return cloud;
}

double TapeDetectorNode::euclideanDistance(const Point2D& p1, const Point2D& p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace tape_detector_ros2