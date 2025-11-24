#include <memory>
#include "my_components/attach_server_component.hpp"
#include "my_components/preapproach_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  //auto pre_approach = std::make_shared<my_components::PreApproach>(options);
  //exec.add_node(pre_approach);

  auto server = std::make_shared<my_components::AttachServer>(options);
  exec.add_node(server);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
