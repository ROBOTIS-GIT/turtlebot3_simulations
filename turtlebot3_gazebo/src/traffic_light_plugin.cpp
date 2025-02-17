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

#include "turtlebot3_gazebo/traffic_light_plugin.hpp"

namespace gazebo
{
TrafficLight::TrafficLight()
: traffic_cycle(5.0),
  status(0),
  textures{"traffic_light_red", "traffic_light_yellow", "traffic_light_green"}
{
}
TrafficLight::~TrafficLight()
{
}
void TrafficLight::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = _model->GetWorld();
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();
  this->visPub = this->node->Advertise<gazebo::msgs::Visual>("/gazebo/default/visual");
  this->msg.set_name(this->model->GetName() + "::traffic_light");
  this->msg.set_parent_name(this->model->GetName());
  std::cout << this->model->GetName() << " plugin load success!" << std::endl;
  this->update_connection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&TrafficLight::OnUpdate, this));
  this->last_time = this->world->SimTime();
}
void TrafficLight::OnUpdate()
{
  common::Time current_time = this->world->SimTime();
  if ((current_time - this->last_time).Double() >= this->traffic_cycle) {
    if (status == 0) {
      status = 1;
      traffic_cycle = 1.0;
    } else if (status == 1) {
      status = 2;
      traffic_cycle = 5.0;
    } else if (status == 2) {
      status = 0;
      traffic_cycle = 5.0;
    }

    msg.mutable_material()->mutable_script()->set_name(textures[status]);

    visPub->Publish(this->msg);
    this->last_time = current_time;
  }
}
}  // namespace gazebo
