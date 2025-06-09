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

#ifndef TURTLEBOT3_GAZEBO__OBSTACLE1_HPP_
#define TURTLEBOT3_GAZEBO__OBSTACLE1_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/math/Vector3.hh>

namespace turtlebot3_gazebo
{

class Obstacle1Plugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  Obstacle1Plugin() = default;
  ~Obstacle1Plugin() override = default;

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;

private:
  gz::sim::Model model;
  std::chrono::steady_clock::time_point startTime;

  std::vector<gz::math::Vector3d> waypoints;
  std::vector<double> segmentDistances;
  double totalDistance = 0.0;
  double speed = 0.1;  // meters per second
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__OBSTACLE1_HPP_
