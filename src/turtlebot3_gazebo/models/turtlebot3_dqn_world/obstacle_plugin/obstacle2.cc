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


namespace gazebo
{
class Obstacle2: public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->model = _parent;

    // create the animation
    gazebo::common::PoseAnimationPtr anim(
      // name the animation "move_2",
      // make it last 260 seconds,
      // and set it on a repeat loop
      new gazebo::common::PoseAnimation("move2", 140.0, true));

    gazebo::common::PoseKeyFrame * key;

    // set starting location of the box
    key = anim->CreateKeyFrame(0);
    key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(10);
    key->Translation(ignition::math::Vector3d(0.7, 0.2, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(40);
    key->Translation(ignition::math::Vector3d(2.5, 3.5, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(55);
    key->Translation(ignition::math::Vector3d(0.3, 3.5, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(85);
    key->Translation(ignition::math::Vector3d(3.5, 1.8, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(100);
    key->Translation(ignition::math::Vector3d(3.5, 0.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(110);
    key->Translation(ignition::math::Vector3d(2.0, 0.5, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(115);
    key->Translation(ignition::math::Vector3d(1.5, 1.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(120);
    key->Translation(ignition::math::Vector3d(1.0, 0.5, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(125);
    key->Translation(ignition::math::Vector3d(0.5, 0.1, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(130);
    key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    key = anim->CreateKeyFrame(140);
    key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

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
GZ_REGISTER_MODEL_PLUGIN(Obstacle2)
}  // namespace gazebo
