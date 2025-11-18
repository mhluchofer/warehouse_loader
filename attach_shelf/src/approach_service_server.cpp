#include <attach_shelf/srv/go_to_loading.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ApproachService : public rclcpp::Node {
public:
  using GoToLoading = attach_shelf::srv::GoToLoading;

  ApproachService() : Node("approach_service") {
    using namespace std::placeholders;

    std::string name_service = "/approach_shelf";
    RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...",
                name_service.c_str());
  }

private:
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto approach_service_node = std::make_shared<ApproachService>();
  rclcpp::spin(approach_service_node);
  rclcpp::shutdown();
  return 0;
}