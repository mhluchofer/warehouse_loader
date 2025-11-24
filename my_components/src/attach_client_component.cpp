#include "my_components/attach_client_component.hpp"
using namespace std::chrono_literals;

namespace my_components
{

AttachClient::AttachClient(const rclcpp::NodeOptions & options)
: Node("attach_client", options)
{
  // Declare parameters
  this->declare_parameter<bool>("attach_to_shelf", true);

  // Service name
  std::string service_name = "/approach_shelf";
  client_ = this->create_client<GoToLoading>(service_name);

  // Wait for the service to come up
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for %s. Exiting.",
                   service_name.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Service %s not available, waiting...",
                service_name.c_str());
  }

  // Timer to call service immediately after creation
  timer_ = this->create_wall_timer(
      0s,
      std::bind(&AttachClient::call_service, this));
}

void AttachClient::call_service()
{
  bool attach_flag = true;
  this->get_parameter("attach_to_shelf", attach_flag);

  auto request = std::make_shared<GoToLoading::Request>();
  request->attach_to_shelf = attach_flag;

  RCLCPP_INFO(this->get_logger(),
              "Calling %s with attach_to_shelf=%s",
              "/approach_shelf",
              attach_flag ? "True" : "False");

  // Async request with callback (non blocking)
  client_->async_send_request(
      request,
      [node = this->shared_from_this()](
          rclcpp::Client<GoToLoading>::SharedFuture future) {

        try {
          auto response = future.get();
          RCLCPP_INFO(node->get_logger(),
                      "Service completed: result=%s",
                      response->complete ? "True" : "False");
        }
        catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(),
                       "Service call failed: %s", e.what());
        }

        // Optional shutdown
        rclcpp::shutdown();
      });

  // Cancel timer to avoid multiple calls
  if (timer_) timer_->cancel();
}

}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
