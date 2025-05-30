// Copyright 2012 Open Source Robotics Foundation
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
// Author: Ryan Shim, ChanHyeong Lee

#include "turtlebot3_gazebo/obstacles.hpp"

#include <cmath>
#include <memory>

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Types.hh>

namespace turtlebot3_gazebo
{

void ObstaclesPlugin::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> &,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  this->model = gz::sim::Model(entity);
  this->startTime = std::chrono::steady_clock::now();
}

void ObstaclesPlugin::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager & ecm)
{
  if (!this->model.Valid(ecm)) {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = now - this->startTime;
  double t = fmod(elapsed.count(), 80.0);
  double angle = 2 * M_PI * t / 80.0;

  gz::math::Pose3d pose(
    gz::math::Vector3d(0, 0, 0),
    gz::math::Quaterniond(0, 0, angle));

  this->model.SetWorldPoseCmd(ecm, pose);
}

}  // namespace turtlebot3_gazebo

GZ_ADD_PLUGIN(
  turtlebot3_gazebo::ObstaclesPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  turtlebot3_gazebo::ObstaclesPlugin,
  "turtlebot3_gazebo::ObstaclesPlugin")
