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
// Author: Ryan Shim

#ifndef TURTLEBOT3_GAZEBO__OBSTACLES_HPP_
#define TURTLEBOT3_GAZEBO__OBSTACLES_HPP_

#include <ignition/math.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#define PI 3.141592

namespace gazebo
{
class Obstacles : public ModelPlugin
{
public:
  Obstacles() = default;
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override;

private:
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;
};
GZ_REGISTER_MODEL_PLUGIN(Obstacles);
}  // namespace gazebo
#endif  // TURTLEBOT3_GAZEBO__OBSTACLES_HPP_
