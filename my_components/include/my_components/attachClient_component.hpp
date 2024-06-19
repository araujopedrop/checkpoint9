#ifndef COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_
#define COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

#include "custom_interfaces/srv/detail/go_to_loading__struct.h"
#include "custom_interfaces/srv/go_to_loading.hpp"

using GoToLoadingServiceMessage = custom_interfaces::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

protected:
  void call_attach_to_shelf_service();
  void response_callback(
      rclcpp::Client<GoToLoadingServiceMessage>::SharedFuture future);

private:
  rclcpp::Client<GoToLoadingServiceMessage>::SharedPtr go_to_loading_client_;
  bool complete_;
};

} // namespace my_components

#endif // COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_