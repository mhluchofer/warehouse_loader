#ifndef MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_
#define MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>
#include <stdexcept>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/utils.h>



namespace my_components {

struct Cluster {
  std::size_t start_idx;
  std::size_t end_idx;
  float value;
};

struct Point {
  double x;
  double y;
  double z;
};


using GoToLoading = attach_shelf::srv::GoToLoading;
//using Point = geometry_msgs::msg::Point;
class ClusterCenterError : public std::runtime_error {
public:
  explicit ClusterCenterError(const std::string &msg)
  : std::runtime_error(msg) {}
};

class AttachServer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

private:
  // ---------------- ROS2 Interfaces ----------------
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr liftup_publisher_;
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;

  // ---------------- TF ----------------
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // ---------------- Data ----------------
  sensor_msgs::msg::LaserScan::SharedPtr scan_;

  // ---------------- Callbacks ----------------
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void attach_callback(const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
                       std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response);

  // ---------------- Algorithm Helpers ----------------
  std::vector<Cluster> find_intensity_clusters(const std::vector<float> &intensities);
  Point find_cluster_center(const sensor_msgs::msg::LaserScan &scan, Cluster &cluster);
  Point find_carte_frame_centre(const Point &p1, const Point &p2);

  void make_cart_frame_tf(const Point &centre_laser, const Point &p1_laser,
                          const Point &p2_laser, const std::string &scan_frame,
                          const rclcpp::Time &stamp);

  void publish_cart_frame_in_odom(const Point &p_odom, double yaw);
  Point transformPointTF2(const geometry_msgs::msg::TransformStamped &T, const Point &p_laser);

  bool run_attach_algorithm();

  inline static bool valid_range(float r) { return std::isfinite(r); }
};

} // namespace my_components

#endif // MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_
