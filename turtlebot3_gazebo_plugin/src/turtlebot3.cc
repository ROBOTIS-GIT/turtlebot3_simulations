/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef GAZEBO_TB3_TURTLEBOT3_PLUGIN_CC_
#define GAZEBO_TB3_TURTLEBOT3_PLUGIN_CC_

#include <fcntl.h>
#include <termios.h>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"

#include <string>
#include <boost/bind.hpp>
#include <iostream>

#define LEFT_WHEEL_JOINT  0
#define RIGHT_WHEEL_JOINT 1

#define LIDAR_SENSOR  1
#define CAMERA_SENSOR 2

#define ESC_ASCII_VALUE   0x1b

using namespace gazebo;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);          
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                       
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  

  int c = getchar();  

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  
  return c;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

class Turtlebot3 : public ModelPlugin
{
 private:
  physics::ModelPtr model_;

  physics::JointPtr left_wheel_joint_;
  physics::JointPtr right_wheel_joint_;

  double wheel_separation_;

  common::PID pid_;

  event::ConnectionPtr updateConnection_;

 public:
  Turtlebot3(){}
  virtual ~Turtlebot3(){}

 public: 
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    getModel(_model);
    
    if (isTurtlebot3Model(_sdf) != true)
      return;

    getJoints(_model);

    getSensors(_model);

    initWheel();

    showMsg();

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Turtlebot3::OnUpdate, this, _1));
  }

 public:
  void getModel(physics::ModelPtr model){ model_ = model; }

 public:
  void getJoints(physics::ModelPtr model)
  {
    left_wheel_joint_  = model->GetJoints()[LEFT_WHEEL_JOINT];
    right_wheel_joint_ = model->GetJoints()[RIGHT_WHEEL_JOINT];

    std::cout << "Find Joint : " << left_wheel_joint_->GetName()  << std::endl;
    std::cout << "Find Joint : " << right_wheel_joint_->GetName() << std::endl;
  }

 public:
  void getSensors(physics::ModelPtr model)
  {
    physics::LinkPtr lidar_link  = model->GetLinks()[LIDAR_SENSOR];
    std::cerr << "Find Sensor : " << lidar_link->GetScopedName() << std::endl;

    if (model_->GetName() != "burger")
    {
      physics::LinkPtr camera_link = model->GetLinks()[CAMERA_SENSOR];
      std::cerr << "Find Sensor : " << camera_link->GetScopedName() << std::endl;
    }
  }

 public:
  bool isTurtlebot3Model(sdf::ElementPtr sdf)
  {
    std::string get_tb3_model;

    if (sdf->HasElement("tb3_model"))
    {
      get_tb3_model = sdf->Get<std::string>("tb3_model");

      if (get_tb3_model == "burger")
      {
        wheel_separation_ = 0.160;
      }
      else if (get_tb3_model == "waffle" || get_tb3_model == "waffle_pi")
      {
        wheel_separation_ = 0.287;
      }
      else
      {
        std::cerr << "Invalid model name, TB3 plugin is not loaded" << std::endl;
        return false;
      }
    }
    else
    {
      std::cerr << "Please put a tb3 model(burger, waffle or waffle_pi) in turtlebot3_.world file, TB3 plugin is not loaded" << std::endl;
      return false;
    }

    std::cout << "The turtlebot3 plugin is attach to model : " << model_->GetName() << "/turtlebot3_" << get_tb3_model << std::endl;

    return true;
  }

 public:
  void writePIDparamForJointControl(double p, double i, double d)
  {
    pid_ = common::PID(p, i, d);

    model_->GetJointController()->SetVelocityPID(left_wheel_joint_->GetScopedName(), pid_);
    model_->GetJointController()->SetVelocityPID(right_wheel_joint_->GetScopedName(), pid_);
  }

 public:
  void initWheel()
  {
    writePIDparamForJointControl(1.0, 0, 0);

    writeVelocityToJoint(0.0, 0.0);
  }
 
 public:
  void showMsg()
  {
    std::cout << " " <<std::endl;
    std::cout << "Control Your TurtleBot3!"       << std::endl;
    std::cout << "---------------------------"    << std::endl;
    std::cout << "w - set linear velocity up"     << std::endl;
    std::cout << "x - set linear velocity down"   << std::endl;
    std::cout << "d - set angular velocity up"    << std::endl;
    std::cout << "a - set angular velocity down"  << std::endl;
    std::cout << "s - set all velocity to zero"   << std::endl;
  }

 public:
  void controlTB3(const float wheel_separation, double lin_vel, double ang_vel)
  {
    // This velocities are not reliable yet due to frinction. It will be updated
    double _lin_vel = lin_vel;
    double _ang_vel = ang_vel;

    double right_wheel_vel = 0.0;
    double left_wheel_vel  = 0.0;

    left_wheel_vel  = _lin_vel - (_ang_vel * wheel_separation / 2);
    right_wheel_vel = _lin_vel + (_ang_vel * wheel_separation / 2);

    writeVelocityToJoint(right_wheel_vel, left_wheel_vel);
  }

 public:
  void writeVelocityToJoint(double right_wheel_vel, double left_wheel_vel)
  {
    model_->GetJointController()->SetVelocityTarget(left_wheel_joint_->GetScopedName(), left_wheel_vel);
    model_->GetJointController()->SetVelocityTarget(right_wheel_joint_->GetScopedName(), right_wheel_vel);
  }
 
 public:
  double getLeftJointPosition()
  {
    return left_wheel_joint_->GetAngle(0).Radian();
  }

 public:
  double getRightJointPosition()
  {
    return right_wheel_joint_->GetAngle(0).Radian();
  }

 public: 
  void OnUpdate(const common::UpdateInfo &)
  {
    static uint8_t hit_cnt = 0;
    static double lin_vel_cmd = 0;
    static double ang_vel_cmd = 0;

    if (kbhit())
    {
      std::cout << " is pressed" << std::endl;
      
      if (hit_cnt > 10)
      {
        showMsg();
        hit_cnt = 0;
      }
      else
        hit_cnt++;

      int c = getch();

      if (c == ESC_ASCII_VALUE)
        exit(0);

      if (c =='w')
        lin_vel_cmd += 0.25;
      else if (c == 'x')
        lin_vel_cmd -= 0.25;
      else if (c == 'a')
        ang_vel_cmd += 0.5;
      else if (c == 'd')
        ang_vel_cmd -= 0.5;
      else if (c == 's')
      {
        lin_vel_cmd = 0;
        ang_vel_cmd = 0;
      }
      else
      {
        lin_vel_cmd = lin_vel_cmd;
        ang_vel_cmd = ang_vel_cmd;
      }

      controlTB3(wheel_separation_, lin_vel_cmd, ang_vel_cmd); 
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(Turtlebot3)

#endif  //GAZEBO_TB3_TURTLEBOT3_PLUGIN_CC_