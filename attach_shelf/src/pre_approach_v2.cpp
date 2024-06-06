#include "custom_interfaces/srv/detail/go_to_loading__struct.h"
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

#include "custom_interfaces/srv/go_to_loading.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;
using GoToLoadingServiceMessage = custom_interfaces::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

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

    this->go_to_loading_client_ =
        this->create_client<GoToLoadingServiceMessage>("/approach_shelf");

    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::publish_velocity, this));

    // ********************************
    //       Parameters setting
    // ********************************

    auto param_desc_obstacle = rcl_interfaces::msg::ParameterDescriptor{};
    auto param_desc_degrees = rcl_interfaces::msg::ParameterDescriptor{};
    auto param_desc_final_approach = rcl_interfaces::msg::ParameterDescriptor{};

    param_desc_obstacle.description =
        "Distance (in meters) to the obstacle at which the robot will stop.";
    param_desc_obstacle.description =
        "Number of degrees for the rotation of the robot after stopping.";

    param_desc_final_approach.description =
        "The value of the request of the service.";

    this->declare_parameter<std::double_t>("obstacle", 0.0,
                                           param_desc_obstacle);
    this->declare_parameter<int>("degrees", 0, param_desc_obstacle);

    this->declare_parameter<bool>("final_approach", false,
                                  param_desc_final_approach);

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

  rclcpp::Client<GoToLoadingServiceMessage>::SharedPtr go_to_loading_client_;

  std::string direction_ = "Forward";

  bool complete_;
  bool service_on_ = false;

  // Parameters variables
  std::double_t obstacle_;
  int degrees_;
  bool final_approach_;

  // PID variables
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
    this->get_parameter("final_approach", final_approach_);

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

      if (service_on_ == false) {
        this->call_attach_to_shelf_service(final_approach_);
        service_on_ = true;
      }
    }

    this->publisher_->publish(velocity);
  }

  void call_attach_to_shelf_service(bool final_approach_arg) {
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

    request->attach_to_shelf = final_approach_arg;

    auto result_future = go_to_loading_client_->async_send_request(
        request, std::bind(&PreApproach::response_callback, this,
                           std::placeholders::_1));
  }

  void response_callback(
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
};

class Approach_service_server : public rclcpp::Node {
public:
  Approach_service_server() : Node("Approach_service_server_node") {

    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    rclcpp::SubscriptionOptions options2;

    options1.callback_group = callback_group_1;
    options2.callback_group = callback_group_2;

    this->go_to_loading_server_ =
        this->create_service<GoToLoadingServiceMessage>(
            "/approach_shelf",
            std::bind(&Approach_service_server::cd_GoToLoading, this, _1, _2));

    this->sub_laserScan_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Approach_service_server::cb_laserScan, this, _1),
            options1);

    this->sub_robot_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Approach_service_server::cb_odom, this, _1),
        options2);

    this->tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "Approach_service_server_node is up!");
  }

private:
  rclcpp::Service<GoToLoadingServiceMessage>::SharedPtr go_to_loading_server_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserScan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_robot_odom_;
  sensor_msgs::msg::LaserScan last_laser_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;

  nav_msgs::msg::Odometry current_robot_pos;

  float angle_degrees_1 = 0.0;
  float angle_degrees_2 = 0.0;

  float frame_pos_x = 0.0;
  float frame_pos_y = 0.0;

  void cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {

    this->current_robot_pos = *msg;
  }

  void cd_GoToLoading(
      const std::shared_ptr<GoToLoadingServiceMessage::Request> request,
      const std::shared_ptr<GoToLoadingServiceMessage::Response> response) {

    int cant_values = 0;

    cant_values = this->get_intensities_values();

    std::cout << "Value requested: " << request->attach_to_shelf;
    std::cout << std::endl;
    std::cout << "Value angle1: " << this->angle_degrees_1;
    std::cout << std::endl;
    std::cout << "Value angle2: " << this->angle_degrees_2;
    std::cout << std::endl;

    this->get_frame_position();
    this->publish_cart_frame();

    if (cant_values == 2) {
      std::cout << std::endl;
      std::cout << "True";
      std::cout << std::endl;
      response->complete = true;
    } else {
      std::cout << std::endl;
      std::cout << "CANT VALUE FOUNDED:" << cant_values;
      std::cout << std::endl;
      response->complete = false;
    }
  }

  void cb_laserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    this->last_laser_ = *msg;
  }

  int get_intensities_values() {

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

  void get_frame_position() {
    float laser1 = 0.0;
    float laser2 = 0.0;
    float laser1_range = 0.0;
    float laser2_range = 0.0;

    float laser1_dist_to_zero_angle = 0.0;
    float laser2_dist_to_zero_angle = 0.0;

    float frame_coordX = 0.0;
    float frame_coordY = 0.0;

    laser1 = 4 * angle_degrees_1 + 540;
    laser2 = 4 * angle_degrees_2 + 540;

    laser1_range = this->last_laser_.ranges[laser1];
    laser2_range = this->last_laser_.ranges[laser2];

    laser1_dist_to_zero_angle =
        std::sin((this->angle_degrees_1 * (180 / M_PI)) * laser1_range);
    laser2_dist_to_zero_angle =
        std::sin((this->angle_degrees_2 * (180 / M_PI)) * laser2_range);

    frame_coordX = (laser1_range + laser2_range) / 2;
    frame_coordY = (laser1_dist_to_zero_angle + laser2_dist_to_zero_angle) / 2;

    std::cout << "La coord [x,y] del frame del docking point es:"
              << frame_coordX << "," << frame_coordY;
    std::cout << std::endl;

    this->frame_pos_x = frame_coordX;
    this->frame_pos_y = frame_coordY;
  }

  void publish_cart_frame() {

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
    t.transform.rotation.x = 1.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 0.0;

    tf_static_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<PreApproach> PreApproach_node =
      std::make_shared<PreApproach>();
  std::shared_ptr<Approach_service_server> PreApproach_v2_node =
      std::make_shared<Approach_service_server>();

  // Multithread

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(PreApproach_node);
  executor.add_node(PreApproach_v2_node);

  executor.spin();

  // 2 Single Thread
  /*
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::executors::StaticSingleThreadedExecutor executor2;

  executor.add_node(PreApproach_node);
  executor2.add_node(PreApproach_v2_node);

  executor.spin();
  executor2.spin();
  */
  rclcpp::shutdown();
  return 0;
}