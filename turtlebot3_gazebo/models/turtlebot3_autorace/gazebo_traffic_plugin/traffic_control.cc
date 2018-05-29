/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 *
*/
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ctime>

namespace gazebo
{
  class TraffiControl : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      // Store the pointer to the model
      this->model = _parent;

       float f = 25.3f;
       printf("start ModelPlugin  ModelPlugin  ModelPlugin  ModelPlugin  ModelPlugin  ModelPlugin load\n");

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&TraffiControl::OnUpdate, this, _1));
	  bar_set_time = time(NULL);

	  physics::Link_V links = model->GetLinks();
      physics::Collision_V colls;
      for (unsigned int i = 0; i < links.size(); i++) {
		  
		  if (links[i]->GetName().compare(light_r) == 0){
		  		light_red_pose = links[i]->GetWorldPose() ;		  	
		  	}else if (links[i]->GetName().compare(light_g) == 0){
		  		light_green_pose = links[i]->GetWorldPose() ;		  	
		  	}else if (links[i]->GetName().compare(light_y) == 0){
		  		light_yellow_pose = links[i]->GetWorldPose() ;		  	
		  	}else if (links[i]->GetName().compare(traffic_bar) == 0){
		  		traffic_bar_pose = links[i]->GetWorldPose() ;
				traffic_bar_pose2 = math::Pose( traffic_bar_pose.pos.x + 0.18, traffic_bar_pose.pos.y ,traffic_bar_pose.pos.z + 0.18
					,traffic_bar_pose.rot.x,traffic_bar_pose.rot.y+1.5,traffic_bar_pose.rot.z ) ;
		  	}

          //cout << links[i]->GetName() << "\t\t" <<links[i]->GetWorldPose() << endl;
          colls = links[i]->GetCollisions();
          for(unsigned i=0; i<colls.size(); i++)
          {
              colls[i]->Update();
              //cout << colls[i]->GetName() << "\t\t" << colls[i]->GetWorldPose() << endl;
          }

      }

      count = 0;	  
	  
	  
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      
	  double     bar_time_diff;
	  double     light_time_diff;

		
	  if( count == 10 ){
	     count = 0;
	  }else{
             count = count + 1;
	     return ;
	  }	
	  
          time_t cur_time = time(NULL) ;
	  bar_time_diff = difftime( cur_time, bar_set_time);

          //printf("start update %f \n " ,bar_time_diff );
	  light_time_diff = difftime( cur_time, light_set_time);	  

	  if( bar_time_diff > bar_duration ){
	  	//printf("start time set..................\n");
	  	if(bar_state==on){			
				bar_state = off ;			
	  		}else if(bar_state==off){	  		    
				bar_state = on ;			
	  		}
			bar_set_time = cur_time;			
	  	}

 	  if(bar_state==on){
		this->model->SetLinkWorldPose( traffic_bar_pose, std::string(traffic_bar));
	  }else if(bar_state==off){
		this->model->SetLinkWorldPose( traffic_bar_pose2, std::string(traffic_bar));
	  }	  


	  if( light_time_diff > light_duration ){
	  	
	  	if(light_state==red){			
				light_state = yellow ;	
	  		}else if(light_state==yellow){	  		
				light_state = green ;
	  		}else if(light_state==green){				
				light_state = red ;	
	  		}
			light_set_time = cur_time;			
	  	}


	  if(light_state==green){
		this->model->SetLinkWorldPose( dispresent, std::string(light_r));
		this->model->SetLinkWorldPose( dispresent2, std::string(light_y));
		this->model->SetLinkWorldPose( light_green_pose, std::string(light_g));
	  }else if(light_state==yellow){
		this->model->SetLinkWorldPose( dispresent, std::string(light_r));
		this->model->SetLinkWorldPose( dispresent2, std::string(light_g));				
		this->model->SetLinkWorldPose( light_yellow_pose, std::string(light_y));
	  }else if(light_state==red){
		this->model->SetLinkWorldPose( dispresent, std::string(light_g));
		this->model->SetLinkWorldPose( dispresent2, std::string(light_y));
		this->model->SetLinkWorldPose( light_red_pose, std::string(light_r));
	  }

    	}

	
    private: time_t bar_set_time;
    private: int on = 0 ;
    private: int off = 1 ;
    private: int bar_state = on;
    private: double bar_duration = 9; // 9sec

    private: time_t light_set_time;
    private: int red = 0 ;
    private: int green = 1 ;
    private: int yellow = 2 ;
    private: int light_state = red;
    private: int count = 0;
    private: double light_duration = 9; // 9sec

    private: std::string light_r = "light_r";
    private: std::string light_g = "light_g";
    private: std::string light_y = "light_y";
    private: std::string traffic_bar = "traffic_bar";

    private: math::Pose light_red_pose = math::Pose( 0,0,0,0,0,0 ) ;
    private: math::Pose light_green_pose = math::Pose( 0,0,0,0,0,0 ) ;
    private: math::Pose light_yellow_pose = math::Pose( 0,0,0,0,0,0 ) ;
    private: math::Pose traffic_bar_pose = math::Pose( 0,0,0,0,0,0 ) ;
    private: math::Pose traffic_bar_pose2 = math::Pose( 0,0,0,0,0,0 ) ;
    private: math::Pose dispresent = math::Pose( 0,0,-0.2,0,0,0 ) ;
    private: math::Pose dispresent2 = math::Pose( 0,1,-0.2,0,0,0 ) ;

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TraffiControl)
}
