#include "my_components/attachServer_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "custom_interfaces/srv/detail/go_to_loading__struct.h"
#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

using GoToLoadingServiceMessage = custom_interfaces::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("AttachServer", options) {

  callback_group_1 =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  callback_group_2 =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions options1;
  rclcpp::SubscriptionOptions options2;

  options1.callback_group = callback_group_1;
  options2.callback_group = callback_group_2;

  this->go_to_loading_server_ = this->create_service<GoToLoadingServiceMessage>(
      "/approach_shelf",
      std::bind(&AttachServer::cd_GoToLoading, this, _1, _2));

  this->sub_laserScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&AttachServer::cb_laserScan, this, _1), options1);

  this->sub_robot_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&AttachServer::cb_odom, this, _1), options2);

  this->tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);

  publisher_elevator_up =
      this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);

  publisher_elevator_down =
      this->create_publisher<std_msgs::msg::Empty>("/elevator_down", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "AttachServer_node is up!");
}

void AttachServer::cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {

  this->current_robot_pos = *msg;
}

// Get transformation between parent_frame and child_frame
geometry_msgs::msg::TransformStamped
AttachServer::get_model_pose_from_tf(const std::string &parent_frame,
                                     const std::string &child_frame) {

  geometry_msgs::msg::TransformStamped ts_;

  ts_.transform.translation.x = 0.0;
  ts_.transform.translation.y = 0.0;
  ts_.transform.translation.z = 0.0;
  ts_.transform.rotation.x = 0.0;
  ts_.transform.rotation.y = 0.0;
  ts_.transform.rotation.z = 0.0;
  ts_.transform.rotation.w = 0.0;

  // Look up for the transformation between parent_frame and child_frame
  try {
    ts_ = tf_buffer_->lookupTransform(parent_frame, child_frame,
                                      tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                parent_frame.c_str(), child_frame.c_str(), ex.what());
    return ts_;
  }

  return ts_;
}

void AttachServer::get_desired_pos() {

  this->desired_pos_ =
      this->get_model_pose_from_tf("robot_base_link", "cart_frame");
}

void AttachServer::wait_for_seconds(int sec) {

  int i = 0;

  rclcpp::Rate rate_(10);

  while (i < 10 * sec) {
    i = i + 1;
    rate_.sleep();
  }
}

void AttachServer::cd_GoToLoading(
    const std::shared_ptr<GoToLoadingServiceMessage::Request> request,
    const std::shared_ptr<GoToLoadingServiceMessage::Response> response) {

  int cant_values = 0;

  float desired_x_ = 0.0;
  float desired_y_ = 0.0;

  float threshold_position = 0.075;
  float threshold_orientation = 0.02;

  float kp_yaw = 0.75;
  float kp_distance = 0.5;

  float speed_approach_docking = 0.3;
  float distance_to_docking = 0.3;

  float error_distance_ = 0.0;
  float error_angle_ = 4.0;

  bool flag_approach_ = false;

  auto start_time = this->now();

  double time_to_move = distance_to_docking / speed_approach_docking;

  std::string direction_ = "GoToCartFrame";

  geometry_msgs::msg::Twist velocity = geometry_msgs::msg::Twist();

  std_msgs::msg::Empty Empty_msg;

  cant_values = this->get_intensities_values();

  if (cant_values == 2) {

    this->get_frame_position();
    this->publish_cart_frame();

    wait_for_seconds(1);

    if (request->attach_to_shelf == true) {

      rclcpp::Rate rate(10);

      while (direction_ != "Done") {

        if (direction_ == "GoToCartFrame") {

          this->get_desired_pos();
          desired_x_ = this->desired_pos_.transform.translation.x;
          desired_y_ = this->desired_pos_.transform.translation.y;

          error_distance_ =
              sqrt(desired_x_ * desired_x_ + desired_y_ * desired_y_);
          error_angle_ = std::atan2(desired_y_, desired_x_);

          if (error_angle_ < -M_PI) {
            error_angle_ += 2 * M_PI;
          } else if (error_angle_ > M_PI) {
            error_angle_ -= 2 * M_PI;
          }

          if (abs(error_distance_) > threshold_position) {

            if (abs(error_angle_) < threshold_orientation) {

              velocity.linear.x = kp_distance * abs(error_distance_);
              velocity.angular.z = 0.0;

            } else {
              velocity.linear.x = kp_distance * abs(error_distance_);
              velocity.angular.z = kp_yaw * error_angle_;
            }

          } else {

            velocity.linear.x = 0.0;
            velocity.angular.z = 0.0;
            direction_ = "Approached";
          }

        } else if (direction_ == "Approached") {

          if (flag_approach_ == false) {
            start_time = this->now();
            flag_approach_ = true;
          }

          if ((this->now() - start_time).seconds() < time_to_move) {
            velocity.linear.x = speed_approach_docking;
            velocity.angular.z = 0.0;
          } else {

            velocity.linear.x = 0.0;
            velocity.angular.z = 0.0;
            direction_ = "Elevator Up";
          }
        } else if (direction_ == "Elevator Up") {

          wait_for_seconds(1);
          publisher_elevator_up->publish(Empty_msg);
          direction_ = "Done";
        }

        publisher_->publish(velocity);
        rate.sleep();
      }
      response->complete = true;
    } else {
      response->complete = false;
    }

  } else {
    response->complete = false;
  }
}

void AttachServer::cb_laserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  this->last_laser_ = *msg;
}

int AttachServer::get_intensities_values() {

  int cant_values = 0;
  float cont_values = 0.0;
  float acu_values = 0.0;
  bool flag = false;

  std::vector<float> intensities_ = this->last_laser_.intensities;

  for (size_t i = 0; i < intensities_.size(); ++i) {
    if (intensities_[i] > 7500) {
      flag = true;
      cont_values = cont_values + 1.0;
      acu_values = acu_values + (float)i;
    } else if (intensities_[i] < 7500 && flag == true) {
      flag = false;
      if (this->angle_degrees_1 == this->angle_degrees_2) {
        this->angle_degrees_1 = ((acu_values / cont_values) - 540) / 4;
      } else {
        this->angle_degrees_2 = ((acu_values / cont_values) - 540) / 4;
      }
      cont_values = 0.0;
      acu_values = 0.0;
      cant_values = cant_values + 1;
    } else if (intensities_[i] > 7500 && i == (intensities_.size() - 1)) {
      cant_values = cant_values + 1;
    }
  }

  return cant_values;
}

void AttachServer::get_frame_position() {

  /*

      Gets the position [x,y] of cart_frame.
      These coordinates are relative to the robot
      For x coordinate I use the average between the beams founded in
     get_intensities_values()


      O       |_       O
       |              |
         |          |
           |      |
             |  |
             Robot

  */

  float laser1 = 0.0;
  float laser2 = 0.0;
  float laser1_range = 0.0;
  float laser2_range = 0.0;

  float laser1_dist_to_zero_angle = 0.0;
  float laser2_dist_to_zero_angle = 0.0;

  float frame_coordX = 0.0;
  float frame_coordX_laser1 = 0.0;
  float frame_coordX_laser2 = 0.0;
  float frame_coordY = 0.0;

  laser1 = 4 * this->angle_degrees_1 + 540;
  laser2 = 4 * this->angle_degrees_2 + 540;

  laser1_range = this->last_laser_.ranges[laser1];
  laser2_range = this->last_laser_.ranges[laser2];

  laser1_dist_to_zero_angle =
      std::sin((this->angle_degrees_1 * (M_PI / 180)) * laser1_range);
  laser2_dist_to_zero_angle =
      std::sin((this->angle_degrees_2 * (M_PI / 180)) * laser2_range);

  frame_coordX_laser1 =
      abs(std::cos((this->angle_degrees_1 * (M_PI / 180)) * laser1_range));
  frame_coordX_laser2 =
      abs(std::cos((this->angle_degrees_2 * (M_PI / 180)) * laser2_range));

  this->dif_pos_scan_base_link =
      this->get_model_pose_from_tf("robot_base_link", "robot_front_laser_base_link");

  frame_coordX = (frame_coordX_laser1 + frame_coordX_laser2) / 2;
  frame_coordY = -(laser1_dist_to_zero_angle + laser2_dist_to_zero_angle) / 2;

  this->frame_pos_x =
      frame_coordX + dif_pos_scan_base_link.transform.translation.x;
  this->frame_pos_y = frame_coordY;
}

void AttachServer::publish_cart_frame() {

  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Quaternion quaternion_;

  double roll, pitch;

  float current_robot_pos_x = 0.0;
  float current_robot_pos_y = 0.0;
  double robot_yaw = 0.0;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "cart_frame";

  current_robot_pos_x = this->current_robot_pos.pose.pose.position.x;
  current_robot_pos_y = this->current_robot_pos.pose.pose.position.y;

  // Quaterion 2 Euler
  quaternion_ = this->current_robot_pos.pose.pose.orientation;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(quaternion_, tf_quaternion);
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, robot_yaw);

  t.transform.translation.x = current_robot_pos_x +
                              this->frame_pos_x * std::cos(robot_yaw) -
                              this->frame_pos_y * std::sin(robot_yaw);
  t.transform.translation.y = current_robot_pos_y +
                              this->frame_pos_x * std::sin(robot_yaw) +
                              this->frame_pos_y * std::cos(robot_yaw);
  t.transform.translation.z = 0.0;

  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = -0.7071;
  t.transform.rotation.w = 0.7071;

  tf_static_broadcaster_->sendTransform(t);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)