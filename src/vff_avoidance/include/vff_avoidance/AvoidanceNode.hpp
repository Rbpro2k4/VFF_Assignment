#ifndef VFF_AVOIDANCE__AVOIDANCE_NODE_HPP_
#define VFF_AVOIDANCE__AVOIDANCE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace vff_avoidance
{

class AvoidanceNode : public rclcpp::Node
{
public:
  explicit AvoidanceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AvoidanceNode();

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timerCallback();
  void computeVectors();
  // Updated publishMarkers declaration with parameters
  void publishMarkers(double attractive_x, double attractive_y,
                      double repulsive_x, double repulsive_y,
                      double result_x, double result_y);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  // Parameters
  std::string laser_topic_;
  std::string cmd_vel_topic_;
  std::string marker_topic_;
  double attractive_gain_;
  double repulsive_gain_;
  double max_linear_speed_;
  double max_angular_speed_;
  double obstacle_distance_threshold_;
  double min_valid_scan_age_;
};

}  // namespace vff_avoidance

#endif  // VFF_AVOIDANCE__AVOIDANCE_NODE_HPP_

