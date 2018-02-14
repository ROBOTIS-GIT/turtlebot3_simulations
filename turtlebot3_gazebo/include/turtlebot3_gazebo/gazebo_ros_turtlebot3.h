/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

#ifndef GAZEBO_ROS_TURTLEBOT3_H_
#define GAZEBO_ROS_TURTLEBOT3_H_

#include <ros/ros.h>
#include <ros/time.h>

#include <math.h>
#include <limits.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

typedef struct
{
  double angle;
  double mag;
  float x;
  float y;
} ScanVector;

class GazeboRosTurtleBot3
{
 public:
  GazeboRosTurtleBot3();
  ~GazeboRosTurtleBot3();
  bool init();
  bool controlLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters
  bool is_debug_;

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_sub_;

  geometry_msgs::Pose pose_;
  double scan_data_[360];
  double tb3_theta_;
  double side_distance_limit_;
  double right_joint_encoder_;
  double priv_right_joint_encoder_;
  double turning_radius_;
  double rotate_angle_;
  double front_distance_limit_;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void imuMsgCallBack(const sensor_msgs::Imu::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr & msg);
};
#endif // GAZEBO_ROS_TURTLEBOT3_H_
