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

#include "turtlebot3_gazebo/gazebo_ros_turtlebot3.h"

GazeboRosTurtleBot3::GazeboRosTurtleBot3()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  ROS_ASSERT(init());
}

GazeboRosTurtleBot3::~GazeboRosTurtleBot3()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool GazeboRosTurtleBot3::init()
{
  // initialize ROS parameter
  std::string robot_model = nh_.param<std::string>("tb3_model", "");
  nh_.param("is_debug", is_debug_, is_debug_);
  tb3_theta_ = 0.0;
  scan_data_[360] = {0.0, };

  if (!robot_model.compare("burger"))
  {
    turning_radius_ = 0.08;
    rotate_angle_ = 50.0 * DEG2RAD;
    front_distance_limit_ = 0.7;
    side_distance_limit_  = 0.4;
  }
  else if (!robot_model.compare("waffle") || !robot_model.compare("waffle_pi"))
  {
    turning_radius_ = 0.1435;
    rotate_angle_ = 40.0 * DEG2RAD;
    front_distance_limit_ = 0.7;
    side_distance_limit_  = 0.6;
  }
  ROS_INFO("robot_model : %s", robot_model.c_str());
  ROS_INFO("turning_radius_ : %lf", turning_radius_);
  ROS_INFO("front_distance_limit_ = %lf", front_distance_limit_);
  ROS_INFO("side_distance_limit_ = %lf", side_distance_limit_);

  // initialize variables
  right_joint_encoder_ = 0.0;
  priv_right_joint_encoder_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("/scan", 10, &GazeboRosTurtleBot3::laserScanMsgCallBack, this);
  imu_sub_         = nh_.subscribe("/imu", 10, &GazeboRosTurtleBot3::imuMsgCallBack, this);
  odom_sub_        = nh_.subscribe("/odom", 10, &GazeboRosTurtleBot3::odomMsgCallBack, this);

  return true;
}

void GazeboRosTurtleBot3::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  pose_ = msg->pose.pose;
}

void GazeboRosTurtleBot3::imuMsgCallBack(const sensor_msgs::Imu::ConstPtr &msg)
{
  float q[4] = {0.0, };

  q[0] = msg->orientation.w;
  q[1] = msg->orientation.x;
  q[2] = msg->orientation.y;
  q[3] = msg->orientation.z;

  tb3_theta_ = atan2f(q[1]*q[2] + q[0]*q[3],
                        0.5f - q[2]*q[2] - q[3]*q[3]);

  tb3_theta_ = tb3_theta_ + 1.57;
}

void GazeboRosTurtleBot3::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  for (int angle = 0; angle < 360; angle++)
  {
    if (std::isinf(msg->ranges.at(angle)))
      scan_data_[angle] = 1.0;
    else
      scan_data_[angle] = (double)(msg->ranges.at(angle) / msg->range_max);
  }
}

void GazeboRosTurtleBot3::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool GazeboRosTurtleBot3::controlLoop()
{
  double lin_vel = 0.2, ang_vel = 0.0;

  ScanVector scanVector[3];

  ScanVector maxVector = {0.0, 1.0, 0.0, 0.0};

  uint16_t index = 0, calc_angle = 0;

//  double error = 0.0;
//  static double pre_error = 0.0;

  // find center max vector
  for (index = 2; index <= 88; index++)
  {
    double mag_ave = 0.0;
    for (int i = -2; i <= 2; i++)
    {
      mag_ave += scan_data_[index+i];
    }
    mag_ave = mag_ave / 5.0f;

    if (maxVector.mag > mag_ave)
    {
      maxVector.mag   = mag_ave;
      maxVector.angle = 90 + index;
    }
  }

//  for (index = 330; index < 360; index++)
//  {
//    if (maxVector.mag < scan_data_[index])
//    {
//      maxVector.mag = scan_data_[index];
//      maxVector.angle = index - 270;
//    }
//  }

//  if (maxVector.angle >= 90)
//  {
//    calc_angle = maxVector.angle - 90;
//    maxVector.x = -1 * sin(calc_angle * DEG2RAD);
//    maxVector.y = cos(calc_angle * DEG2RAD);
//  }
//  else
//  {
//    maxVector.x = cos(maxVector.angle * DEG2RAD);
//    maxVector.y = sin(maxVector.angle * DEG2RAD);
//  }

  calc_angle = maxVector.angle - 90;
  maxVector.x = -1 * sin(calc_angle * DEG2RAD);
  maxVector.y = cos(calc_angle * DEG2RAD);
  scanVector[LEFT] = maxVector;

  // find left max vector
//  maxVector = {0.0, 0.0, 0.0, 0.0};
//  for (index = 30; index < 90; index++)
//  {
//    if (maxVector.mag < scan_data_[index])
//    {
//      maxVector.mag = scan_data_[index];
//      maxVector.angle = 90 + index;
//    }
//  }

//  calc_angle = maxVector.angle - 90;
//  maxVector.x = -1 * sin(calc_angle * DEG2RAD);
//  maxVector.y = cos(calc_angle * DEG2RAD);
//  scanVector[LEFT] = maxVector;

  // find right max vector
  maxVector = {0.0, 1.0, 0.0, 0.0};
//  for (index = 270; index < 330; index++)
//  {
//    if (maxVector.mag < scan_data_[index])
//    {
//      maxVector.mag = scan_data_[index];
//      maxVector.angle = index - 270;
//    }
//  }

  for (index = 357; index >= 272; index--)
  {
    double mag_ave = 0.0;
    for (int i = -2; i <= 2; i++)
    {
      mag_ave += scan_data_[index+i];
    }
    mag_ave = mag_ave / 5.0f;

    if (maxVector.mag > mag_ave)
    {
      maxVector.mag   = mag_ave;
      maxVector.angle = index - 270;
    }
  }

  maxVector.x = cos(maxVector.angle * DEG2RAD);
  maxVector.y = sin(maxVector.angle * DEG2RAD);
  scanVector[RIGHT] = maxVector;

  ROS_INFO("angle : %.2f, %.2f, %.2f, mag : %.2f, %.2f, %.2f", scanVector[LEFT].angle, 0.0, scanVector[RIGHT].angle,
                                                               scanVector[LEFT].mag, 0.0, scanVector[RIGHT].mag);

//  ROS_INFO("angle : %.2f, %.2f, %.2f, x : %.2f, %.2f, %.2f, y : %.2f, %.2f, %.2f",
//                                                         scanVector[LEFT].angle, scanVector[CENTER].angle, scanVector[RIGHT].angle,
//                                                         scanVector[LEFT].x, scanVector[CENTER].x, scanVector[RIGHT].x,
//                                                         scanVector[LEFT].y, scanVector[CENTER].y, scanVector[RIGHT].y);

  // find main vector
//  maxVector = {0.0, 0.0, 0.0, 0.0};
//  for (index = 0; index < 2; index++)
//  {
//    maxVector.x += scanVector[index].x;
//    maxVector.y += scanVector[index].y;
//  }

//  maxVector.angle = atan2(maxVector.y, maxVector.x);

//  double odom_orientation = tf::getYaw(pose_.orientation);
//  odom_orientation = odom_orientation + 1.57;

//  error = (tb3_theta_ + 1.57) - maxVector.angle;
//  error = maxVector.angle - odom_orientation;

//  ang_vel = 1.5 * error +
//            0.0 * (error - pre_error) / 0.008;

//  pre_error = error;

//  maxVector.angle = 1.57 - maxVector.angle;

//  ang_vel = -1 * (maxVector.angle * RAD2DEG) * 0.06f;

//  ROS_INFO("vector angle : %.2f, robot ori : %.2f, odom ori : %.2f, ang_vel : %.2f",
//           maxVector.angle * RAD2DEG, tb3_theta_ * RAD2DEG, odom_orientation * RAD2DEG, ang_vel);

//  ROS_INFO("vector angle : %.2f, ang_vel : %.2f",
//           maxVector.angle * RAD2DEG, ang_vel);

  if (scanVector[LEFT].mag < 0.2)
    ang_vel = -2.0;
  else if (scanVector[RIGHT].mag < 0.2)
    ang_vel = 2.0;
  else
    ang_vel = 0.0;

  updatecommandVelocity(lin_vel, ang_vel);

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gazebo_ros_turtlebot3");
  GazeboRosTurtleBot3 gazeboRosTurtleBot3;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    gazeboRosTurtleBot3.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
