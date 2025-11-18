#include <attach_shelf/srv/go_to_loading.hpp>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>

struct Cluster {
  std::size_t start_idx;
  std::size_t end_idx;
  float value;
};

class ApproachService : public rclcpp::Node {

public:
    using GoToLoading = attach_shelf::srv::GoToLoading;

    ApproachService() : Node("approach_service") {

        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ApproachService::scan_callback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 10);

        liftup_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/elevator_up", 10);

        // Transform members
        tf_static_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        service_ = this->create_service<GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachService::approach_callback, this, 
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "approach_shelf service ready!");
    }

private:
    // ---------------- VARIABLES ----------------
    sensor_msgs::msg::LaserScan::SharedPtr last_scan;
    rclcpp::Service<GoToLoading>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr liftup_publisher_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>  tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ------------ SCAN CALLBACK -----------
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan = msg;
    }

    // ------------------- SERVICE -------------------
    void approach_callback(const std::shared_ptr<GoToLoading::Request> request,
                            std::shared_ptr<GoToLoading::Response> response) {
        const bool attach_to_shelf = request->attach_to_shelf;
        RCLCPP_INFO(this->get_logger(),
                    "Approach Service Requested, AttachToShelf: %s",
                    attach_to_shelf ? "True" : "False");

        std::vector<Cluster> intensity_clusters =
            find_intensity_clusters(last_scan->intensities);

        if (intensity_clusters.size() != 2) {
        RCLCPP_ERROR(this->get_logger(),
                    "Detected %ld shelf legs(clusters), while need 2 shelf "
                    "legs(clusters)",
                    intensity_clusters.size());
        response->complete = false;
        return;
        }
        RCLCPP_INFO(this->get_logger(), "Detected %ld shelf legs(clusters)",intensity_clusters.size());

        // 2. Identify centres of cluster and calculate centre between 2 shelf legs
        geometry_msgs::msg::Point p1 = find_cluster_center(*last_scan, intensity_clusters[0]);
        geometry_msgs::msg::Point p2 = find_cluster_center(*last_scan, intensity_clusters[1]);
        RCLCPP_INFO(this->get_logger(), "First leg X: %.3f Y: %.3f", p1.x, p1.y);
        RCLCPP_INFO(this->get_logger(), "Second leg X: %.3f Y: %.3f", p2.x, p2.y);

        geometry_msgs::msg::Point centre;
        centre.x = 0.5 * (p1.x + p2.x);
        centre.y = 0.5 * (p1.y + p2.y);
        RCLCPP_INFO(this->get_logger(), "Centre point X: %.3f Y: %.3f", centre.x,
                    centre.y);

        // 3. Publish static transform cart_frame with parent frame odom
        make_cart_frame_tf(centre);
        RCLCPP_INFO(this->get_logger(), "Published 'cart_frame' transform. ");

        if (!attach_to_shelf) {
        RCLCPP_INFO(this->get_logger(),
                    "Approach Service completed successfully!");
        response->complete = true;
        return;
        }

        // 4. Attach to shelf
        if (run_attach_algorithm()) {
        RCLCPP_INFO(this->get_logger(),"Approach Service completed successfully!");
        response->complete = true;
        } else {
        response->complete = false;
        RCLCPP_INFO(this->get_logger(), "Approach Service failed!");
        }
    }


    // -------- FIND INTENSITY CLUSTERS --------
    std::vector<Cluster> find_intensity_clusters(const std::vector<float> &intensities) {
        std::vector<Cluster> clusters;
        if (intensities.empty())
        return clusters;

        std::size_t start = 0;
        float current = intensities[0];

        auto keep = [](float v) { return v != 0.0f; };

        for (std::size_t i = 1; i < intensities.size(); i++) {
        if (current != intensities[i]) {
            if (keep(current)) {
            clusters.push_back({start, i - 1, current});
            }
            start = i;
            current = intensities[i];
        }
        }
        if (keep(current)) {
        clusters.push_back({start, intensities.size() - 1, current});
        }
        return clusters;
    }

    // -------- CLUSTER CENTER --------------
    geometry_msgs::msg::Point
    find_cluster_center(const sensor_msgs::msg::LaserScan &scan,
                                Cluster &cluster) {
        float sx = 0.0f, sy = 0.0f, count = 0.0f;
        for (auto i = cluster.start_idx; i <= cluster.end_idx; i++) {
        float r = scan.ranges[i];
        if (!valid_range(r))
            continue;

        const float theta = scan.angle_min + i * scan.angle_increment;
        sx += r * std::cos(theta);
        sy += r * std::sin(theta);
        count += 1.0;
        }

        if (count == 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Can't identify cluster center.");
        // TODO: Write exception throwing
        //   throw
        }

        geometry_msgs::msg::Point p;
        p.x = sx / count;
        p.y = sy / count;
        return p;
    }

    // -------- PUBLISH TF --------
    void make_cart_frame_tf(geometry_msgs::msg::Point &point_laser) {
        // 1. Look up transform odom <- laser (we use latest)
        geometry_msgs::msg::TransformStamped T;

        try {
        T = tf_buffer_->lookupTransform(
            "odom",                        // target frame
            "robot_front_laser_base_link", // source frame (laser)
            rclcpp::Time(0),               // time of the scan
            rclcpp::Duration::from_seconds(0.1));
        } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
        }

        // 2. Transform our target point to odom
        geometry_msgs::msg::Point point_odom;
        point_odom = transformPointManual(T, point_laser);
        // point_odom = transfromPointTF2(T, point_laser);

        // 3. Publish the static frame in odom
        publish_cart_frame_in_odom(point_odom);
    }

    // --------- MOTION TO CART FRAME ---------
    bool run_attach_algorithm() {

        const std::string robot_frame = "robot_base_footprint";
        const std::string cart_frame = "cart_frame";

        const double kp_dist = 0.5;
        const double kp_yaw = 2.0;
        const double v_min = 0.1, v_max = 0.5; // min and max linear velocity
        const double w_max = 1.0; // min and max angular velocity
        const double stop_dist = 0.02, stop_yaw = 0.02; // stop distance and yaw

        rclcpp::Rate rate(10.0); // 10 Hz (100ms per iteration)

        // 1. Use TF to approach cart_frame
        while (rclcpp::ok()) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            // Look up for the transformation between laser and cart frames
            tf = tf_buffer_->lookupTransform(robot_frame, cart_frame,
                                            rclcpp::Time(0),
                                            rclcpp::Duration::from_seconds(0.1));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(),
                        "TF robot_front_laser_base_link to cart_frame failed: %s",
                        ex.what());
            return false;
        }

        const double dx = tf.transform.translation.x;
        const double dy = tf.transform.translation.y;
        const double error_distance = std::hypot(dx, dy);
        const double error_yaw = std::atan2(dy, dx);
        RCLCPP_INFO(this->get_logger(), "Distance to cart: %.3f  Yaw: %.3f",
                    error_distance, error_yaw);
        if (error_distance <= stop_dist && std::abs(error_yaw) <= stop_yaw) {
            geometry_msgs::msg::Twist stop;
            cmd_vel_publisher_->publish(stop);
            RCLCPP_INFO(this->get_logger(), "Reached the cart frame. (%.3f m)",
                        error_distance);
            break;
        }

        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = std::clamp(kp_dist * error_distance, v_min, v_max);
        double w = kp_yaw * error_yaw;
        if (w > w_max)
            w = w_max;
        if (w < -w_max)
            w = -w_max;
        cmd.angular.z = w;
        cmd_vel_publisher_->publish(cmd);
        rate.sleep();
        }

        // 2. Do additional 30cm forward using only cart_frame
        while (rclcpp::ok()) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            // Look up for the transformation between laser and cart frames
            tf = tf_buffer_->lookupTransform(robot_frame, cart_frame,
                                            rclcpp::Time(0),
                                            rclcpp::Duration::from_seconds(0.1));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(),
                        "TF robot_front_laser_base_link to cart_frame failed: %s",
                        ex.what());
            return false;
        }

        const double dx = tf.transform.translation.x;
        const double dy = tf.transform.translation.y;
        const double error_distance = std::hypot(dx, dy);
        RCLCPP_INFO(this->get_logger(), "Distance to cart_frame: %.3f",
                    error_distance);
        if (error_distance >= 0.3) {
            geometry_msgs::msg::Twist stop;
            cmd_vel_publisher_->publish(stop);
            RCLCPP_INFO(this->get_logger(), "Advance 30cm complete.");
            break;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v_min;
        cmd_vel_publisher_->publish(cmd);
        rate.sleep();
        }

        // 3. Rise up the shelf
        std_msgs::msg::String msg;
        liftup_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Lifted the shelf up.");
        return true;
    }

    void publish_cart_frame_in_odom(const geometry_msgs::msg::Point &p_odom) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "cart_frame";
        tf.transform.translation.x = p_odom.x;
        tf.transform.translation.y = p_odom.y;
        tf.transform.translation.z = p_odom.z;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(tf);
    }

    geometry_msgs::msg::Point transformPointManual(
        const geometry_msgs::msg::TransformStamped &laser_odom_tf,
        const geometry_msgs::msg::Point &p_laser) {
        // q - rotation from laser axes to odom axes  || R{odom<-laser}
        tf2::Quaternion q(
            laser_odom_tf.transform.rotation.x, laser_odom_tf.transform.rotation.y,
            laser_odom_tf.transform.rotation.z, laser_odom_tf.transform.rotation.w);
        // t - translation - origin of laser in odom coordinates  || t{odom<-laser}
        tf2::Vector3 t(laser_odom_tf.transform.translation.x,
                    laser_odom_tf.transform.translation.y,
                    laser_odom_tf.transform.translation.z);
        // Now any point in laser frame coordinates system to be in odom coordinates
        // frame calculates by formula:
        // p_odom = R{odom<-laser}*p_laser + t{odom<-laser}
        // target point is our point in laser coordinate frame
        tf2::Vector3 vec_laser(p_laser.x, p_laser.y, p_laser.z);

        // Apply rotation then translation:  p_odom = R(q)*p_laser + t
        tf2::Vector3 vec_odom = tf2::quatRotate(q, vec_laser) + t;

        geometry_msgs::msg::Point p_odom;
        p_odom.x = vec_odom.x();
        p_odom.y = vec_odom.y();
        p_odom.z = vec_odom.z();
        return p_odom;
    }

    geometry_msgs::msg::Point transfromPointTF2(
        const geometry_msgs::msg::TransformStamped &T,
        const geometry_msgs::msg::Point &p_laser) 
    {
        geometry_msgs::msg::PointStamped ps_laser, ps_odom;
        ps_laser.header.frame_id = "robot_front_laser_base_link";
        ps_laser.header.stamp = this->get_clock()->now();
        ps_laser.point = p_laser;

        tf2::doTransform(ps_laser, ps_odom, T);
        return ps_odom.point;
    }
    inline bool valid_range(float r) { return std::isfinite(r); }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto approach_service_node = std::make_shared<ApproachService>();
  rclcpp::spin(approach_service_node);
  rclcpp::shutdown();
  return 0;
}
