#include <attach_shelf/srv/go_to_loading.hpp>
#include "rclcpp/logging.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:

    using GoToLoading = attach_shelf::srv::GoToLoading;
    PreApproach() : Node("pre_approach_node") {
    using rcl_interfaces::msg::FloatingPointRange;
    using rcl_interfaces::msg::IntegerRange;
    using rcl_interfaces::msg::ParameterDescriptor;

    // --- obstacle distance (m) ---
    ParameterDescriptor obstacle_desc;
    obstacle_desc.description = "Distance to stop before obstacle (meters).";
    FloatingPointRange d_range;
    d_range.from_value = 0.1; // 5 cm minimum
    d_range.to_value = 10.0;  // 10 m maximum
    d_range.step = 0.0;       // any value in range
    obstacle_desc.floating_point_range = {d_range};

    // --- rotation (deg) ---
    ParameterDescriptor degrees_desc;
    degrees_desc.description = "Rotation to apply after stopping (degrees).";
    IntegerRange deg_range;
    deg_range.from_value = -180;
    deg_range.to_value = 180;
    deg_range.step = 1;
    degrees_desc.integer_range = {deg_range};

    // Declare parameters (with defualts)
    this->declare_parameter<double>("obstacle", 0.5,
                                    obstacle_desc); // Obstacle distance to
                                                    // stop (m)
    this->declare_parameter<int>(
        "degrees", 0, degrees_desc); // Rotation degrees after stop (degrees)

    // Load params for use
    this->get_parameter("obstacle", obstacle_dist_);
    obstacle_dist_ += obstacle_error_; // small error to detect early obstacle

    this->get_parameter("degrees", rotation_degrees_);

    RCLCPP_INFO(this->get_logger(),
                "Params loaded: obstacle=%.2f m, degrees=%.d", obstacle_dist_,
                rotation_degrees_);

    // Subscribers
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproach::laserscan_callback, this,
                  std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    // Services
    std::string name_service = '/approach_shelf';
    approach_client_ = this->create_client<GoToLoading>(name_service);

    // Wait for the service to be available (check every second)
    while (!approach_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service %s not available, waiting again...",
                  name_service.c_str());
    }

    // Control Loop Timer
    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::control_loop, this));

    approaching_ = true; // start forward
    turning_ = false;
    front_range_ = std::numeric_limits<float>::infinity();
    RCLCPP_INFO(this->get_logger(), "Pre Approach V2 Node ready!");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  //rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<GoToLoading> approach_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  double obstacle_dist_{0.5}; // Param: stop distance
  int rotation_degrees_{0};   // Param: rotation after stop
  const double obstacle_error_ =
      0.04; // considering 10HZ and 0.5 speed,(give or take 1 sec err)
  const double linear_vel_ = 0.5;
  const double fast_angular_vel_ = 0.5;
  const double slow_angular_vel_ = 0.2;
  const double slow_zone_rad_ = 15.0 * M_PI / 180;
  double current_yaw_, target_yaw_;
  float front_range_;
  bool approaching_, turning_;

  void control_loop() {
    geometry_msgs::msg::Twist cmd;

    if (approaching_) {
      //   RCLCPP_INFO(this->get_logger(), "Front Range: %.3f", front_range_);
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

        double yaw_error = normalize_angle(target_yaw_ - current_yaw_);

        // 1. Condición de parada
        if (std::abs(yaw_error) < 0.05) {
            stop();
            turning_ = false;
            rclcpp::shutdown();
            return;
        }

        // 2. Control proporcional
        double k_p = 2.5;   // Ajustable: más grande = gira más fuerte
        double angular_speed = k_p * yaw_error;

        // 3. Límite de seguridad
        angular_speed = std::clamp(angular_speed, -fast_angular_vel_, fast_angular_vel_);

        // 4. Aplicar comando
        cmd.angular.z = angular_speed;

        // Log opcional
        RCLCPP_INFO(this->get_logger(),
            "Turning P ctrl | yaw: %.3f  target: %.3f  error: %.3f  w: %.3f",
            current_yaw_, target_yaw_, yaw_error, angular_speed);
    }

    cmd_vel_publisher_->publish(cmd);
  }

  void stop() {
    geometry_msgs::msg::Twist zero;
    cmd_vel_publisher_->publish(zero);
  }

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    current_yaw_ = tf2::getYaw(q);
  }

  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto pre_approach_node = std::make_shared<PreApproach>();
  rclcpp::spin(pre_approach_node);
  rclcpp::shutdown();
  return 0;
}