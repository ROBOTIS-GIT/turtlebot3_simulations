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
void imageCallbackMsg(ConstImageStampedPtr &msg)
{
  std::cout << "image width : "  << msg->image().width()  << std::endl;
  std::cout << "image height : " << msg->image().height() << std::endl;

  char *data;
  data = new char[msg->image().data().length() + 1];

  memcpy(data, msg->image().data().c_str(), msg->image().data().length());

  // Add your code

  delete data;
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
    std::cerr << "Please put a tb3 model(`waffle` or `waffle_pi`)" << std::endl;
    exit(0);
  }

  char* tb3_model = _argv[1];
  std::string topic_name = "";

  if (std::string(tb3_model) == "waffle")
    topic_name = "/gazebo/default/user/turtlebot3_waffle/image/intel_realsense_r200/image";
  else if (std::string(tb3_model) == "waffle_pi")
    topic_name = "/gazebo/default/user/turtlebot3_waffle_pi/image/raspberry_pi_cam/image";
  else 
  {
    std::cerr << "Please put a tb3 model(`waffle` or `waffle_pi`)" << std::endl;
    exit(0);
  }

  std::cout << "topic name is " << topic_name << std::endl;

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe(topic_name, imageCallbackMsg);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
