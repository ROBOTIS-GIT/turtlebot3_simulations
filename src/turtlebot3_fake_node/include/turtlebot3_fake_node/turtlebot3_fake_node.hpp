// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Yoonseok Pyo, Ryan Shim

#ifndef TURTLEBOT3_FAKE_NODE__TURTLEBOT3_FAKE_NODE_HPP_
#define TURTLEBOT3_FAKE_NODE__TURTLEBOT3_FAKE_NODE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "turtlebot3_msgs/msg/sensor_state.hpp"

#define LEFT 0
#define RIGHT 1

class Turtlebot3Fake : public rclcpp::Node
{
public:
  Turtlebot3Fake();
  ~Turtlebot3Fake();

private:
  // ROS time
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time prev_update_time_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // ROS topic publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;


  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::JointState joint_states_;

  double wheel_speed_cmd_[2];
  double goal_linear_velocity_;
  double goal_angular_velocity_;
  double cmd_vel_timeout_;
  double last_position_[2];
  double last_velocity_[2];
  float odom_pose_[3];
  float odom_vel_[3];

  double wheel_seperation_;
  double wheel_radius_;

  // Function prototypes
  void init_parameters();
  void init_variables();
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  void update_callback();
  bool update_odometry(const rclcpp::Duration & diff_time);
  void update_joint_state();
  void update_tf(geometry_msgs::msg::TransformStamped & odom_tf);
};
#endif  // TURTLEBOT3_FAKE_NODE__TURTLEBOT3_FAKE_NODE_HPP_
