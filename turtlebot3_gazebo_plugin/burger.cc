
#ifndef GAZEBO_TB3_BURGER_PLUGIN_CC_
#define GAZEBO_TB3_BURGER_PLUGIN_CC_

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

class Burger : public ModelPlugin
{
 public:
  Burger(){}

 public: 
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
  {
    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Invalid joint count, TB3 Burger plugin not loaded\n";
      return;
    }

    this->model = _model;

    this->joint = _model->GetJoint("left_wheel_joint");

    this->pid = common::PID(0.1, 0, 0);

    this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

    this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 10.0);
  }

 private:
  physics::ModelPtr model;
  physics::JointPtr joint;
  common::PID pid;
};

GZ_REGISTER_MODEL_PLUGIN(Burger)

#endif