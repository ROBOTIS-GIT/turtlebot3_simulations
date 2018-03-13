/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
 *
*/

/* Authors: Taehun Lim (Darby) */

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void laserScanCallbackMsg(ConstLaserScanStampedPtr &msg)
{
  std::cout << "min angle : " << msg->scan().angle_min() << std::endl;
  std::cout << "max angle : " << msg->scan().angle_max() << std::endl;
  std::cout << "min range : " << msg->scan().range_min() << std::endl;
  std::cout << "max range : " << msg->scan().range_max() << std::endl;

  std::cout << "scan data : [";
  for (int angle = 0; angle < msg->scan().ranges_size(); angle++)
  {
    std::cout << msg->scan().ranges(angle) << ", ";
  }

  std::cout << "]" << std::endl;

  // Add your code
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  if (_argc < 2)
  {
    std::cerr << "Please put a tb3 model(`burger`, `waffle` or `waffle_pi`)" << std::endl;
    exit(0);
  }

  char* tb3_model = _argv[1];
  std::string topic_name = "/gazebo/default/user/turtlebot3_" + std::string(tb3_model) + "/lidar/hls_lfcd_lds/scan";

  std::cout << "topic name is " << topic_name << std::endl;

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe(topic_name, laserScanCallbackMsg);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
