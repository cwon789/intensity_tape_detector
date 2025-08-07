#ifndef TAPE_DETECTOR_ROS2__TAPE_DETECTOR_NODE_HPP_
#define TAPE_DETECTOR_ROS2__TAPE_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

namespace tape_detector_ros2
{

struct Point2D
{
  double x;
  double y;
  float intensity;
  
  Point2D(double x_val = 0.0, double y_val = 0.0, float i_val = 0.0)
    : x(x_val), y(y_val), intensity(i_val) {}
};

struct Cluster
{
  std::vector<Point2D> points;
  Point2D centroid;
  
  void calculateCentroid()
  {
    if (points.empty()) {
      centroid = Point2D(0.0, 0.0, 0.0);
      return;
    }
    
    double sum_x = 0.0, sum_y = 0.0;
    float sum_intensity = 0.0;
    
    for (const auto& p : points) {
      sum_x += p.x;
      sum_y += p.y;
      sum_intensity += p.intensity;
    }
    
    centroid.x = sum_x / points.size();
    centroid.y = sum_y / points.size();
    centroid.intensity = sum_intensity / points.size();
  }
};

class TapeDetectorNode : public rclcpp::Node
{
public:
  explicit TapeDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TapeDetectorNode() = default;

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
  std::vector<Point2D> preprocessAndFilter(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
  std::vector<Cluster> clusterHighIntensityPoints(const std::vector<Point2D>& points);
  
  void performEuclideanClustering(
    const std::vector<Point2D>& points,
    std::vector<Cluster>& clusters,
    double tolerance);
  
  Point2D findLowIntensityFeature(
    const Point2D& hi_centroid,
    const std::vector<Point2D>& all_points);
  
  void publishPointClouds(
    const std::vector<Cluster>& hi_clusters,
    const std::vector<Point2D>& li_features);
  
  sensor_msgs::msg::PointCloud2 createPointCloud(
    const std::vector<Point2D>& points,
    const std::string& frame_id);
  
  double euclideanDistance(const Point2D& p1, const Point2D& p2);
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hi_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr li_cloud_pub_;
  
  double min_range_;
  double max_range_;
  double intensity_threshold_;
  double cluster_tolerance_;
  int min_cluster_size_;
  double search_radius_;
};

}  // namespace tape_detector_ros2

#endif  // TAPE_DETECTOR_ROS2__TAPE_DETECTOR_NODE_HPP_