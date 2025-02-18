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

#include "turtlebot3_gazebo/obstacles.hpp"

namespace gazebo
{
void Obstacles::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  this->model = _parent;

  gazebo::common::PoseAnimationPtr anim(
    new gazebo::common::PoseAnimation("move", 40.0, true));

  gazebo::common::PoseKeyFrame * key;

  key = anim->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = anim->CreateKeyFrame(20);
  key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, PI));

  key = anim->CreateKeyFrame(40);
  key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 2 * PI));

  _parent->SetAnimation(anim);
}
}  // namespace gazebo
