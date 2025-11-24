#ifndef MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_
#define MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_


#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <attach_shelf/srv/go_to_loading.hpp>
namespace my_components
{

using GoToLoading = attach_shelf::srv::GoToLoading;

class AttachClient : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions & options);

private:
  void call_service();

  rclcpp::Client<GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace my_components

#endif  // MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_
