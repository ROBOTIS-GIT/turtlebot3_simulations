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

#include "turtlebot3_gazebo/traffic_bar_plugin.hpp"

namespace gazebo
{
TrafficBar::TrafficBar()
: traffic_cycle(10.0),
  status(0),
  down_pose(
    ignition::math::Vector3d(-0.85, 1.26, 0.125),
    ignition::math::Quaterniond(0.0, 0.0, -1.57)),
  up_pose(
    ignition::math::Vector3d(-0.75, 1.11, 0.275),
    ignition::math::Quaterniond(0.0, -1.57, -1.57))
{
}
void TrafficBar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = _model->GetWorld();

  std::cout << this->model->GetName() << " plugin load success!" << std::endl;
  this->update_connection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&TrafficBar::OnUpdate, this));

  this->last_time = this->world->SimTime();
  this->model->SetGravityMode(false);
}
void TrafficBar::OnUpdate()
{
  common::Time current_time = this->world->SimTime();
  if ((current_time - this->last_time).Double() >= this->traffic_cycle) {
    this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
    this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
    this->model->SetStatic(false);

    if (status == 0) {
      this->model->SetWorldPose(up_pose);
      status = 1;
    } else {
      this->model->SetWorldPose(down_pose);
      status = 0;
    }
    this->last_time = current_time;
  }
}
}  // namespace gazebo
