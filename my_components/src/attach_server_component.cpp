#include "my_components/attach_server_component.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("attach_server_node", options) {
  using namespace std::placeholders;

  // Subscribers
  scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&AttachServer::laserscan_callback, this, _1));

  // Publishers
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);
  liftup_publisher_ =
      this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

  // TF
  tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Service
  service_ = this->create_service<GoToLoading>(
      "/approach_shelf", std::bind(&AttachServer::attach_callback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "AttachServer ready, service /approach_shelf running.");
}

// ------------------- CALLBACKS -------------------
void AttachServer::laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  scan_ = msg;
}

void AttachServer::attach_callback(
    const std::shared_ptr<GoToLoading::Request> request,
    std::shared_ptr<GoToLoading::Response> response) {

  if (!scan_) {
    RCLCPP_ERROR(get_logger(), "No LaserScan received yet");
    response->complete = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Attach requested: %s",
              request->attach_to_shelf ? "True" : "False");

  auto clusters = find_intensity_clusters(scan_->intensities);
  if (clusters.size() != 2) {
    RCLCPP_ERROR(get_logger(), "Expected 2 clusters, found %ld", clusters.size());
    response->complete = false;
    return;
  }

  // Cluster centers
  Point p1 = find_cluster_center(*scan_, clusters[0]);
  Point p2 = find_cluster_center(*scan_, clusters[1]);
  Point centre = find_carte_frame_centre(p1, p2);

  // Publish cart frame with orientation
  make_cart_frame_tf(centre, p1, p2, scan_->header.frame_id, scan_->header.stamp);

  if (!request->attach_to_shelf) {
    RCLCPP_INFO(get_logger(), "Attach skipped, service completed.");
    response->complete = true;
    return;
  }

  // Run attach algorithm
  response->complete = run_attach_algorithm();
}

// ------------------- AUXILIAR METHODS -------------------
std::vector<Cluster> AttachServer::find_intensity_clusters(const std::vector<float> &intensities) {
  std::vector<Cluster> clusters;
  if (intensities.empty()) return clusters;

  size_t start = 0;
  float current = intensities[0];
  auto keep = [](float v){ return v != 0.0f; };

  for (size_t i=1; i<intensities.size(); ++i) {
    if (intensities[i] != current) {
      if (keep(current)) clusters.push_back({start, i-1, current});
      start = i; current = intensities[i];
    }
  }
  if (keep(current)) clusters.push_back({start, intensities.size()-1, current});
  return clusters;
}

Point AttachServer::find_cluster_center(const sensor_msgs::msg::LaserScan &scan, Cluster &cluster) {
  float sx=0.0, sy=0.0, count=0.0;
  for(size_t i=cluster.start_idx; i<=cluster.end_idx; ++i){
    float r = scan.ranges[i];
    if (!valid_range(r)) continue;
    float theta = scan.angle_min + i*scan.angle_increment;
    sx += r*cos(theta); sy += r*sin(theta);
    count += 1.0;
  }
  if (count == 0.0) throw ClusterCenterError("No valid rays in cluster");
  return {sx/count, sy/count, 0.0};
}

// Ajuste nudge y orientación
Point AttachServer::find_carte_frame_centre(const Point &p1, const Point &p2){
  Point centre{0.5*(p1.x+p2.x), 0.5*(p1.y+p2.y), 0.0};
  double dx = p2.x - p1.x, dy = p2.y - p1.y;
  double len = std::hypot(dx, dy);
  double nudge = std::clamp(std::hypot(p2.x,p2.y) - std::hypot(p1.x,p1.y), -0.04, 0.04);
  centre.x += nudge * dx/len; centre.y += nudge * dy/len;
  return centre;
}

void AttachServer::make_cart_frame_tf(const Point &centre_laser, const Point &p1_laser, const Point &p2_laser,
                                      const std::string &scan_frame, const rclcpp::Time &stamp) {
  geometry_msgs::msg::TransformStamped T;
  try { T = tf_buffer_->lookupTransform("odom", scan_frame, stamp, rclcpp::Duration::from_seconds(0.1)); }
  catch(const tf2::TransformException &ex) { RCLCPP_WARN(get_logger(), "TF failed: %s", ex.what()); return; }

  Point centre_odom = transformPointTF2(T, centre_laser);
  Point p1_odom = transformPointTF2(T, p1_laser);
  Point p2_odom = transformPointTF2(T, p2_laser);

  double yaw = std::atan2(p2_odom.y - p1_odom.y, p2_odom.x - p1_odom.x) + M_PI/2.0;
  publish_cart_frame_in_odom(centre_odom, yaw);
}

Point AttachServer::transformPointTF2(const geometry_msgs::msg::TransformStamped &T, const Point &p_laser){
  PointStamped ps_laser, ps_odom;
  ps_laser.header.frame_id = T.child_frame_id;
  ps_laser.header.stamp = T.header.stamp;
  ps_laser.point = p_laser;
  tf2::doTransform(ps_laser, ps_odom, T);
  return ps_odom.point;
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)
