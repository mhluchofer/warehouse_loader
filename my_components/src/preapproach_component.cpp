#include "my_components/preapproach_component.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

using namespace std::chrono_literals;

static double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}

namespace my_components {

PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("pre_approach_node", options) {

  // Subscribers
  scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PreApproach::laserscan_callback, this, std::placeholders::_1));

  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

  // Publishers
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  // Control Loop Timer
  timer_ = this->create_wall_timer(100ms,
                                   std::bind(&PreApproach::control_loop, this));

  approaching_ = true; // start forward
  turning_ = false;
  front_range_ = std::numeric_limits<float>::infinity();

  RCLCPP_INFO(this->get_logger(), "Pre Approach Node ready!");
}

void PreApproach::control_loop() {
  geometry_msgs::msg::Twist cmd;

  if (approaching_) {
    if (front_range_ < obstacle_dist_) {
      // Stop forward
      stop();
      approaching_ = false;
      turning_ = true;

      const double turn_radians = rotation_degrees_ * (M_PI / 180);
      target_yaw_ = normalize_angle(current_yaw_ + turn_radians);
      RCLCPP_INFO(this->get_logger(),
                  "Stopping at obstacle. Turning %d deg (current yaw: "
                  "%.3f, target yaw: %.3f)",
                  rotation_degrees_, current_yaw_, target_yaw_);
    } else {
      cmd.linear.x = linear_vel_;
    }
  } else if (turning_) {
    const double yaw_error = normalize_angle(target_yaw_ - current_yaw_);
    if (std::abs(yaw_error) < 0.01) { // ~0.6°
      stop();
      turning_ = false;
      RCLCPP_INFO(this->get_logger(), "Turn complete! Final yaw: %.3f rad",
                  current_yaw_);
      rclcpp::shutdown();
      return;
    }
    double angular_speed = std::abs(yaw_error) > slow_zone_rad_
                               ? fast_angular_vel_
                               : slow_angular_vel_;
    cmd.angular.z = std::copysign(angular_speed, yaw_error);
  }

  cmd_vel_publisher_->publish(cmd);
}

void PreApproach::stop() {
  geometry_msgs::msg::Twist zero;
  cmd_vel_publisher_->publish(zero);
}

void PreApproach::laserscan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  const double half = 10.0 * M_PI / 180; // 10 degrees front
  const size_t right_idx =
      std::ceil((-half - msg->angle_min) / msg->angle_increment);
  const size_t left_idx =
      std::floor((half - msg->angle_min) / msg->angle_increment) + 1;
  float min_val = std::numeric_limits<float>::infinity();
  for (auto i = right_idx; i < left_idx; ++i) {
    min_val = std::min(min_val, msg->ranges[i]);
  }
  front_range_ = min_val;
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  current_yaw_ = tf2::getYaw(q);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)