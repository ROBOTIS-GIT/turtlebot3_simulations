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
  nh_.param("is_debug", is_debug_, is_debug_);
  tb3_theta_ = 0.0;
  scan_data_[360] = {0.0, };

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("/scan", 10, &GazeboRosTurtleBot3::laserScanMsgCallBack, this);
  imu_sub_         = nh_.subscribe("/imu", 10, &GazeboRosTurtleBot3::imuMsgCallBack, this);

  return true;
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

  ScanVector maxVector = {0.0, 0.0, 0.0, 0.0};

  uint16_t angle = 0, calc_angle = 0;

  double error = 0.0;
  static double pre_error = 0.0;

  // find center max vector
  for (angle = 0; angle < 45; angle++)
  {
    if (maxVector.mag < scan_data_[angle])
    {
      maxVector.mag = scan_data_[angle];
      maxVector.angle = angle;
    }
  }

  for (angle = 315; angle < 360; angle++)
  {
    if (maxVector.mag < scan_data_[angle])
    {
      maxVector.mag = scan_data_[angle];
      maxVector.angle = angle;
    }
  }

  if (maxVector.angle > 315)
  {
    calc_angle = maxVector.angle - 270;
    maxVector.x = -1 * sin(calc_angle * RAD2DEG);
    maxVector.y = cos(calc_angle * RAD2DEG);
  }
  else
  {
    maxVector.x = cos(maxVector.angle * RAD2DEG);
    maxVector.y = sin(maxVector.angle * RAD2DEG);
  }
  scanVector[CENTER] = maxVector;

  // find right max vector
  maxVector = {0.0, 0.0, 0.0, 0.0};
  for (angle = 45; angle < 90; angle++)
  {
    if (maxVector.mag < scan_data_[angle])
    {
      maxVector.mag = scan_data_[angle];
      maxVector.angle = angle;
    }
  }

  maxVector.x = cos(maxVector.angle * RAD2DEG);
  maxVector.y = sin(maxVector.angle * RAD2DEG);
  scanVector[RIGHT] = maxVector;

  // find left max vector
  maxVector = {0.0, 0.0, 0.0, 0.0};
  for (angle = 270; angle < 315; angle++)
  {
    if (maxVector.mag < scan_data_[angle])
    {
      maxVector.mag = scan_data_[angle];
      maxVector.angle = angle;
    }
  }

  calc_angle = maxVector.angle - 270;
  maxVector.x = -1 * sin(calc_angle * RAD2DEG);
  maxVector.y = cos(calc_angle * RAD2DEG);
  scanVector[LEFT] = maxVector;

  // find main vector
  maxVector = {0.0, 0.0, 0.0, 0.0};
  for (int index = 0; index < 3; index++)
  {
    maxVector.x += scanVector[index].x;
    maxVector.y += scanVector[index].y;
  }

  maxVector.angle = atan2(maxVector.y, maxVector.x);

  error = tb3_theta_ - maxVector.angle;

  ang_vel = 0.7 * error +
            0.0 * (error - pre_error) / 0.008;

  pre_error = error;

//  torque[PAN]  = p_gain_ * error[PAN] +
//                 d_gain_ * ((error[PAN] - pre_error[PAN]) / 0.004);
//  torque[TILT] = p_gain_ * error[TILT] +
//                 d_gain_ * ((error[TILT] - pre_error[TILT]) / 0.004) +
//                 tilt_motor_mass * gravity * link_length * cos(convertValue2Radian((int32_t)motorPos_->cur_pos.at(TILT)));

//  ROS_INFO("center_ave : %.3f, right_ave : %.3f, left_ave : %.3f", center_ave, right_ave, left_ave);

//  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3],
//                  0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);


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

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    gazeboRosTurtleBot3.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
