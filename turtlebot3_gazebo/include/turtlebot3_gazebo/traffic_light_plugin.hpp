// Copyright 2025 ROBOTIS CO., LTD.
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
// Author: Hyungyu Kim

#ifndef TURTLEBOT3_GAZEBO__TRAFFIC_LIGHT_PLUGIN_HPP_
#define TURTLEBOT3_GAZEBO__TRAFFIC_LIGHT_PLUGIN_HPP_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class TrafficLight : public ModelPlugin
{
public:
  TrafficLight();
  ~TrafficLight();

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate();

private:
  double traffic_cycle;
  int status;
  std::vector<std::string> textures;

  common::Time last_time;
  event::ConnectionPtr update_connection;
  gazebo::transport::NodePtr node;
  gazebo::msgs::Visual msg;
  gazebo::transport::PublisherPtr visPub;
  physics::ModelPtr model;
  physics::WorldPtr world;
};
GZ_REGISTER_MODEL_PLUGIN(TrafficLight);
}  // namespace gazebo

#endif  // TURTLEBOT3_GAZEBO__TRAFFIC_LIGHT_PLUGIN_HPP_
