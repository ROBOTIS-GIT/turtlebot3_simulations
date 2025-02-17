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

#include "turtlebot3_gazebo/obstacle1.hpp"

namespace gazebo
{
void Obstacle1::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  this->model = _parent;

  gazebo::common::PoseAnimationPtr anim(
    new gazebo::common::PoseAnimation("move1", 160.0, true));

  gazebo::common::PoseKeyFrame * key;

  key = anim->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = anim->CreateKeyFrame(10);
  key->Translation(ignition::math::Vector3d(-0.5, -1.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = anim->CreateKeyFrame(50);
  key->Translation(ignition::math::Vector3d(-3.5, -1.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = anim->CreateKeyFrame(70);
  key->Translation(ignition::math::Vector3d(-3.7, -3.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = anim->CreateKeyFrame(90);
  key->Translation(ignition::math::Vector3d(-3.5, -1.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = anim->CreateKeyFrame(130);
  key->Translation(ignition::math::Vector3d(-0.5, -1.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = anim->CreateKeyFrame(140);
  key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = anim->CreateKeyFrame(160);
  key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  _parent->SetAnimation(anim);
}
}  // namespace gazebo
