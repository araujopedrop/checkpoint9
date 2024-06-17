#include "my_components/preApproach_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace my_components {
PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("PreApproach", options) {

  callback_group_1 =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  callback_group_2 =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions options1;
  rclcpp::SubscriptionOptions options2;

  options1.callback_group = callback_group_1;
  options2.callback_group = callback_group_2;

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  this->sub_laserScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&PreApproach::cb_laserScan, this, _1), options1);

  this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PreApproach::cb_odom, this, _1), options2);

  timer_ = this->create_wall_timer(
      100ms, std::bind(&PreApproach::publish_velocity, this));

  // ******************************** Parameters setting
  // ********************************

  this->obstacle_ = 0.3;
  this->degrees_ = -90;

  RCLCPP_INFO(this->get_logger(), "preApproach_node is up!");
}

void PreApproach::cb_laserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  this->last_laser_ = *msg;
}

void PreApproach::cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {

  geometry_msgs::msg::Quaternion quaternion_;

  double roll, pitch, yaw;

  // Quaterion 2 Euler
  quaternion_ = msg->pose.pose.orientation;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(quaternion_, tf_quaternion);
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

  this->current_angle_ = yaw;
}

void PreApproach::publish_velocity() {

  if (this->last_laser_.ranges[540] > obstacle_ && direction_ == "Forward") {
    direction_ = "Forward";
  } else if (this->last_laser_.ranges[540] < obstacle_ &&
             direction_ == "Forward") {
    direction_ = "Turn";
  } else if (abs(error_angle_) < 0.1 && direction_ == "Turn") {
    direction_ = "Stop";
  }

  if (direction_ == "Forward") {
    velocity.linear.x = 0.5;
    velocity.angular.z = 0.0;
  } else if (direction_ == "Turn") {

    velocity.linear.x = 0.0;

    error_angle_ = degrees_ * (M_PI / 180) - current_angle_;

    if (error_angle_ < -M_PI) {
      error_angle_ += 2 * M_PI;
    } else if (error_angle_ > M_PI) {
      error_angle_ -= 2 * M_PI;
    }

    velocity.angular.z = kp_ * error_angle_;

  } else if (direction_ == "Stop") {

    velocity.linear.x = 0.0;
    velocity.angular.z = 0.0;
  }

  this->publisher_->publish(velocity);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)