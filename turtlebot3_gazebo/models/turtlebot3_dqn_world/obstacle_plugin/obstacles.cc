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

#include <ignition/math.hh>
#include <stdio.h>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#define PI 3.141592


namespace gazebo
{
class Obstacles: public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->model = _parent;

    // create the animation
    gazebo::common::PoseAnimationPtr anim(
      // name the animation "move",
      // make it last 40 seconds,
      // and set it on a repeat loop
      new gazebo::common::PoseAnimation("move", 40.0, true));

    gazebo::common::PoseKeyFrame * key;

    // set starting location of the box
    key = anim->CreateKeyFrame(0);
    key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(20);
    key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, PI));

    key = anim->CreateKeyFrame(40);
    key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 2 * PI));

    // set the animation
    _parent->SetAnimation(anim);
  }

// Pointer to the model

private:
  physics::ModelPtr model;

// Pointer to the update event connection

private:
  event::ConnectionPtr updateConnection;
};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Obstacles)
}  // namespace gazebo
