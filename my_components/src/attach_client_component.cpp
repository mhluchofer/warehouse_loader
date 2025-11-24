#include "my_components/attach_client_component.hpp"

using namespace std::chrono_literals;

namespace my_components
{

AttachClient::AttachClient(const rclcpp::NodeOptions & options)
: Node("attach_client", options)
{
  client_ = this->create_client<GoToLoading>("/approach_shelf");

  // Call the service once the system is ready
  this->declare_parameter<bool>("attach_to_shelf", true);
  this->declare_parameter<double>("timeout", 5.0);

  // Wait a little before calling
  rclcpp::TimerBase::SharedPtr timer_ =
    this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AttachClient::call_service, this));
}

void AttachClient::call_service()
{
  if (!client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(),
      "Service /approach_shelf not available...");
    return;
  }

  auto request = std::make_shared<GoToLoading::Request>();
  this->get_parameter("attach_to_shelf", request->attach_to_shelf);

  RCLCPP_INFO(this->get_logger(),
    "Calling /approach_shelf attach_to_shelf=%s",
    request->attach_to_shelf ? "true" : "false");

  auto future = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        future,
        std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(),
      "Service completed: result=%s",
      future.get()->complete ? "true" : "false");
  } else {
    RCLCPP_ERROR(this->get_logger(),
      "Service call failed or timeout");
  }
}

}  // namespace my_components
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
