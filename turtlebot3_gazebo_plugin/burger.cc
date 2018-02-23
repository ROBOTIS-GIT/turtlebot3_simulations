#ifndef GAZEBO_TB3_BURGER_PLUGIN_CC_
#define GAZEBO_TB3_BURGER_PLUGIN_CC_

#include <fcntl.h>
#include <termios.h>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

#include <boost/bind.hpp>

using namespace gazebo;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

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

    std::cerr << "The burger plugin is attach to model : " << this->model->GetName() << "\n";

    this->left_wheel_joint = _model->GetJoints()[1];
    this->right_wheel_joint = _model->GetJoints()[2];
    
    std::cerr << "Get Joint : " << this->left_wheel_joint->GetName()  << "\n";
    std::cerr << "Get Joint : " << this->right_wheel_joint->GetName() << "\n";

    this->pid = common::PID(1.0, 0, 0);

    this->model->GetJointController()->SetVelocityPID(this->left_wheel_joint->GetScopedName(), this->pid);
    this->model->GetJointController()->SetVelocityPID(this->right_wheel_joint->GetScopedName(), this->pid);

    updateConnection = event::Events::ConnectWorldUpdateBegin(
										boost::bind(&Burger::OnUpdate,
										this, _1));
  }

 public: 
  void OnUpdate(const common::UpdateInfo &)
  {
    // int c = getch();

    // if (c == 'w')
    // {
    //   this->model->GetJointController()->SetVelocityTarget(this->left_wheel_joint->GetScopedName(), 10.0);
    //   this->model->GetJointController()->SetVelocityTarget(this->right_wheel_joint->GetScopedName(), 10.0);
    // }
    // else if (c == 'x')
    // {
    //   this->model->GetJointController()->SetVelocityTarget(this->left_wheel_joint->GetScopedName(), -10.0);
    //   this->model->GetJointController()->SetVelocityTarget(this->right_wheel_joint->GetScopedName(), -10.0);      
    // }

    this->model->GetJointController()->SetVelocityTarget(this->left_wheel_joint->GetScopedName(), -10.0);
    this->model->GetJointController()->SetVelocityTarget(this->right_wheel_joint->GetScopedName(), -10.0); 
  }

 private:
  physics::ModelPtr model;

  physics::JointPtr left_wheel_joint;
  physics::JointPtr right_wheel_joint;

  common::PID pid;

  event::ConnectionPtr updateConnection;
  double wheel_vel;
};

GZ_REGISTER_MODEL_PLUGIN(Burger)

#endif