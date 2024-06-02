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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("preApproach_node") {

    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    rclcpp::SubscriptionOptions options2;

    options1.callback_group = callback_group_1;
    options2.callback_group = callback_group_2;

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    this->sub_laserScan_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PreApproach::cb_laserScan, this, _1),
            options1);

    this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PreApproach::cb_odom, this, _1), options2);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::publish_velocity, this));

    // ******************************** Parameters setting
    // ********************************

    auto param_desc_obstacle = rcl_interfaces::msg::ParameterDescriptor{};
    auto param_desc_degrees = rcl_interfaces::msg::ParameterDescriptor{};

    param_desc_obstacle.description =
        "Distance (in meters) to the obstacle at which the robot will stop.";
    param_desc_obstacle.description =
        "Number of degrees for the rotation of the robot after stopping.";

    this->declare_parameter<std::double_t>("obstacle", 0.0,
                                           param_desc_obstacle);
    this->declare_parameter<int>("degrees", 0, param_desc_obstacle);

    RCLCPP_INFO(this->get_logger(), "preApproach_node is up!");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserScan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  sensor_msgs::msg::LaserScan last_laser_;
  nav_msgs::msg::Odometry odom_value_;

  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;
  geometry_msgs::msg::Twist velocity = geometry_msgs::msg::Twist();

  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;

  std::string direction_ = "Forward";

  std::double_t obstacle_;
  int degrees_;

  float current_angle_ = 0.0;
  float desired_angle_ = 0.0;
  float error_angle_ = 4.0;
  float kp_ = 0.75;

  void cb_laserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    this->last_laser_ = *msg;
  }

  void cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {

    geometry_msgs::msg::Quaternion quaternion_;

    double roll, pitch, yaw;

    // Quaterion 2 Euler
    quaternion_ = msg->pose.pose.orientation;

    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion_, tf_quaternion);
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    this->current_angle_ = yaw;
  }

  void publish_velocity() {

    this->get_parameter("obstacle", obstacle_);
    this->get_parameter("degrees", degrees_);

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
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}