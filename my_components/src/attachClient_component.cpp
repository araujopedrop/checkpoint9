#include "my_components/attachClient_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "custom_interfaces/srv/detail/go_to_loading__struct.h"
#include "custom_interfaces/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

using GoToLoadingServiceMessage = custom_interfaces::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("AttachClient", options) {

  this->go_to_loading_client_ =
      this->create_client<GoToLoadingServiceMessage>("/approach_shelf");

  this->call_attach_to_shelf_service();

  RCLCPP_INFO(this->get_logger(), "AttachClient_node is up!");
}

void AttachClient::call_attach_to_shelf_service() {
  while (!go_to_loading_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for service. Terminating...");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Service Unavailable. Waiting for Service...");
  }

  auto request = std::make_shared<GoToLoadingServiceMessage::Request>();

  request->attach_to_shelf = true;

  auto result_future = go_to_loading_client_->async_send_request(
      request,
      std::bind(&AttachClient::response_callback, this, std::placeholders::_1));
}

void AttachClient::response_callback(
    rclcpp::Client<GoToLoadingServiceMessage>::SharedFuture future) {
  auto status = future.wait_for(1s);
  if (status == std::future_status::ready) {
    auto result = future.get();
    this->complete_ = result->complete;
    RCLCPP_INFO(this->get_logger(), "Result: %s",
                this->complete_ ? "true" : "false");
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)