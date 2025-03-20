#include "vff_avoidance/AvoidanceNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <chrono>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

namespace vff_avoidance
{

AvoidanceNode::AvoidanceNode(const rclcpp::NodeOptions & options)
: Node("avoidance_node", options)
{
  // Declare and get parameters from YAML configuration
  this->declare_parameter<std::string>("laser_topic", "/scan");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<std::string>("marker_topic", "/vff_markers");
  this->declare_parameter<double>("timer_frequency", 20.0);
  this->declare_parameter<double>("attractive_gain", 1.0);
  this->declare_parameter<double>("repulsive_gain", 1.5);
  this->declare_parameter<double>("max_linear_speed", 0.5);
  this->declare_parameter<double>("max_angular_speed", 1.0);
  this->declare_parameter<double>("obstacle_distance_threshold", 1.5);
  this->declare_parameter<double>("min_valid_scan_age", 2.0);

  laser_topic_ = this->get_parameter("laser_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  marker_topic_ = this->get_parameter("marker_topic").as_string();
  double timer_frequency = this->get_parameter("timer_frequency").as_double();
  attractive_gain_ = this->get_parameter("attractive_gain").as_double();
  repulsive_gain_ = this->get_parameter("repulsive_gain").as_double();
  max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
  obstacle_distance_threshold_ = this->get_parameter("obstacle_distance_threshold").as_double();
  min_valid_scan_age_ = this->get_parameter("min_valid_scan_age").as_double();

  // Create subscriptions and publishers
  laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_topic_, 10,
    std::bind(&AvoidanceNode::laserCallback, this, std::placeholders::_1));

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

  // Create timer to run the algorithm at 20 Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / timer_frequency)),
    std::bind(&AvoidanceNode::timerCallback, this));
}

AvoidanceNode::~AvoidanceNode()
{
}

void AvoidanceNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Store the latest laser scan
  latest_scan_ = msg;
}

void AvoidanceNode::timerCallback()
{
  // Check if we have a valid laser scan message
  if (!latest_scan_) {
//    RCLCPP_WARN(this->get_logger(), "No laser scan received yet.");
    return;
  }

  // Check if the laser scan is too old
  auto now = this->now();
  auto scan_time = rclcpp::Time(latest_scan_->header.stamp);
  if ((now - scan_time).seconds() > min_valid_scan_age_) {
//    RCLCPP_WARN(this->get_logger(), "Laser scan is too old.");
    return;
  }

  // Compute control commands based on VFF algorithm
  computeVectors();
}

void AvoidanceNode::computeVectors()
{
  // Attractive vector: always forward (along positive X)
  double attractive_x = attractive_gain_;
  double attractive_y = 0.0;

  // Repulsive vector: computed from laser scan data
  double repulsive_x = 0.0;
  double repulsive_y = 0.0;
  bool obstacle_found = false;
  double closest_distance = std::numeric_limits<double>::infinity();
  double angle_of_closest = 0.0;

  int num_readings = latest_scan_->ranges.size();
  double angle_min = latest_scan_->angle_min;
  double angle_increment = latest_scan_->angle_increment;

  // Process laser scan to find the closest obstacle within threshold
  for (int i = 0; i < num_readings; ++i) {
    double range = latest_scan_->ranges[i];
    double angle = angle_min + i * angle_increment;
    if (std::isfinite(range) && range < obstacle_distance_threshold_ && range < closest_distance) {
      closest_distance = range;
      angle_of_closest = angle;
      obstacle_found = true;
    }
  }

  if (obstacle_found && closest_distance > 0.0) {
    // Repulsive vector inversely proportional to the distance
    double repulsive_magnitude = repulsive_gain_ / closest_distance;
    repulsive_x = -repulsive_magnitude * std::cos(angle_of_closest);
    repulsive_y = -repulsive_magnitude * std::sin(angle_of_closest);
  }

  // Sum attractive and repulsive vectors to compute the resultant vector
  double result_x = attractive_x + repulsive_x;
  double result_y = attractive_y + repulsive_y;

  // Slow down if an obstacle is near: use the ratio (closest_distance / threshold) as a speed factor.
  double speed_factor = 1.0;
  if (obstacle_found) {
    speed_factor = std::min(closest_distance / obstacle_distance_threshold_, 1.0);
  }
  double linear_speed = max_linear_speed_ * speed_factor;

  // Compute angular speed based on resultant vector's angle
  double angular_speed = std::atan2(result_y, result_x);
  if (angular_speed > max_angular_speed_) {
    angular_speed = max_angular_speed_;
  } else if (angular_speed < -max_angular_speed_) {
    angular_speed = -max_angular_speed_;
  }

  // Publish the twist command
  geometry_msgs::msg::Twist cmd_msg;
  cmd_msg.linear.x = linear_speed;
  cmd_msg.angular.z = angular_speed;
  cmd_vel_publisher_->publish(cmd_msg);

  // Publish debugging markers so that RViz can display the vectors.
  if (marker_publisher_->get_subscription_count() > 0) {
    publishMarkers(attractive_x, attractive_y, repulsive_x, repulsive_y, result_x, result_y);
  }
}

void AvoidanceNode::publishMarkers(double attractive_x, double attractive_y,
                                   double repulsive_x, double repulsive_y,
                                   double result_x, double result_y)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Marker for Attractive vector (blue arrow)
  visualization_msgs::msg::Marker attractive_marker;
  attractive_marker.header.frame_id = "base_link";
  attractive_marker.header.stamp = this->now();
  attractive_marker.ns = "attractive";
  attractive_marker.id = 0;
  attractive_marker.type = visualization_msgs::msg::Marker::ARROW;
  attractive_marker.action = visualization_msgs::msg::Marker::ADD;
  attractive_marker.pose.orientation.w = 1.0;
  attractive_marker.scale.x = 0.1; // Arrow length
  attractive_marker.scale.y = 0.000125; // Shaft diameter
  attractive_marker.scale.z = 0.005; // Head diameter
  attractive_marker.color.r = 0.0;
  attractive_marker.color.g = 0.0;
  attractive_marker.color.b = 1.0;
  attractive_marker.color.a = 1.0;
  attractive_marker.points.resize(2);
  attractive_marker.points[0].x = 0.0;
  attractive_marker.points[0].y = 0.0;
  attractive_marker.points[0].z = 0.0;
  attractive_marker.points[1].x = attractive_x;
  attractive_marker.points[1].y = attractive_y;
  attractive_marker.points[1].z = 0.0;
  marker_array.markers.push_back(attractive_marker);

  // Marker for Repulsive vector (red arrow)
  visualization_msgs::msg::Marker repulsive_marker;
  repulsive_marker.header.frame_id = "base_link";
  repulsive_marker.header.stamp = this->now();
  repulsive_marker.ns = "repulsive";
  repulsive_marker.id = 1;
  repulsive_marker.type = visualization_msgs::msg::Marker::ARROW;
  repulsive_marker.action = visualization_msgs::msg::Marker::ADD;
  repulsive_marker.pose.orientation.w = 1.0;
  repulsive_marker.scale.x = 0.1;  // arrow length
  repulsive_marker.scale.y = 0.000125; // shaft diameter
  repulsive_marker.scale.z = 0.005; // head diameter
  repulsive_marker.color.r = 1.0;
  repulsive_marker.color.g = 0.0;
  repulsive_marker.color.b = 0.0;
  repulsive_marker.color.a = 1.0;
  repulsive_marker.points.resize(2);
  repulsive_marker.points[0].x = 0.0;
  repulsive_marker.points[0].y = 0.0;
  repulsive_marker.points[0].z = 0.0;
  repulsive_marker.points[1].x = repulsive_x;
  repulsive_marker.points[1].y = repulsive_y;
  repulsive_marker.points[1].z = 0.0;
  marker_array.markers.push_back(repulsive_marker);

  // Marker for Resultant vector (green arrow)
  visualization_msgs::msg::Marker result_marker;
  result_marker.header.frame_id = "base_link";
  result_marker.header.stamp = this->now();
  result_marker.ns = "result";
  result_marker.id = 2;
  result_marker.type = visualization_msgs::msg::Marker::ARROW;
  result_marker.action = visualization_msgs::msg::Marker::ADD;
  result_marker.pose.orientation.w = 1.0;
  result_marker.scale.x = 0.1;  // arrow length
  result_marker.scale.y = 0.000125; // shaft diameter
  result_marker.scale.z = 0.005; // head diameter
  result_marker.color.r = 0.0;
  result_marker.color.g = 1.0;
  result_marker.color.b = 0.0;
  result_marker.color.a = 1.0;
  result_marker.points.resize(2);
  result_marker.points[0].x = 0.0;
  result_marker.points[0].y = 0.0;
  result_marker.points[0].z = 0.0;
  result_marker.points[1].x = result_x;
  result_marker.points[1].y = result_y;
  result_marker.points[1].z = 0.0;
  marker_array.markers.push_back(result_marker);

  marker_publisher_->publish(marker_array);
}

}  // namespace vff_avoidance
