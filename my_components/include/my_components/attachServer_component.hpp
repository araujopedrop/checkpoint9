#ifndef COMPOSITION__ATTACHSERVER_COMPONENT_HPP_
#define COMPOSITION__ATTACHSERVER_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

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
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/buffer.h>

using GoToLoadingServiceMessage = custom_interfaces::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

protected:
  void cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  geometry_msgs::msg::TransformStamped
  get_model_pose_from_tf(const std::string &parent_frame,
                         const std::string &child_frame);
  void get_desired_pos();
  void wait_for_seconds(int sec);
  void cd_GoToLoading(
      const std::shared_ptr<GoToLoadingServiceMessage::Request> request,
      const std::shared_ptr<GoToLoadingServiceMessage::Response> response);
  void cb_laserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  int get_intensities_values();
  void get_frame_position();
  void publish_cart_frame();

private:
  rclcpp::Service<GoToLoadingServiceMessage>::SharedPtr go_to_loading_server_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserScan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_robot_odom_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_elevator_up;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_elevator_down;

  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_msgs::msg::Odometry current_robot_pos;
  sensor_msgs::msg::LaserScan last_laser_;
  geometry_msgs::msg::TransformStamped desired_pos_;
  geometry_msgs::msg::TransformStamped dif_pos_scan_base_link;

  float angle_degrees_1 = 0.0;
  float angle_degrees_2 = 0.0;

  float frame_pos_x = 0.0;
  float frame_pos_y = 0.0;

  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace my_components

#endif // COMPOSITION__ATTACHSERVER_COMPONENT_HPP_