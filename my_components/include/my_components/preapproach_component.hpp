#ifndef COMPOSITION__PREAPPROACH_COMPONENT_HPP_
#define COMPOSITION__PREAPPROACH_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace my_components {

class PreApproach : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions &options);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  const double obstacle_dist_ = 0.3 + 0.05; // Stop distance + 1 sec error 10HZ
  const int rotation_degrees_ = -90;        // Rotation after stop
  const double linear_vel_ = 0.5;
  const double fast_angular_vel_ = 0.5;
  const double slow_angular_vel_ = 0.2;
  const double slow_zone_rad_ = 15.0 * M_PI / 180;
  double current_yaw_, target_yaw_;
  float front_range_;
  bool approaching_, turning_;
   bool exit_on_end_{true}; // Exit program when finished parameter

  void control_loop();
  void stop();
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

} // namespace my_components

#endif // COMPOSITION__PREAPPROACH_COMPONENT_HPP_