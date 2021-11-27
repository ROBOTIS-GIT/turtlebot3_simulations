#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Pose2D

import os
import datetime as date
from functools import partial

class params:
	def __init__(self):
		#   RL params
		self.num_Epi = 50000
		self.steps_per_Epi = 1000
		self.epsilon = 1.0
		self.lr = 0.0001
		self.gamma = 0.99
		self.eps_dec = 0.00000001
		self.epsilon_min = 0.01
		self.mem_size = 30000
		self.batch_size = 64
		self.replace_target = 1000
		self.chkpt_dir = os.getcwd()+'/models/'
		self.save_string = '_'

		#   Simulation params
		self.dt = 0.1

		#   Environment params
		self.boundary_min = 0
		self.boundary_max = 100
		self.num_L = 1
		self.L_type = 'L'
		self.a_type = 'SA'
		self.num_SA = 50
		self.init_patch_radius = 1.5*(self.num_SA+self.num_L)
		self.init_patch_center = [50,50] #np.random.uniform(self.boundary_min+self.init_patch_radius,
                                          #          self.boundary_max-self.init_patch_radius,2)
		self.Rr = 2
		self.Ro = 11
		self.Ra = 26
        # self.Rr = 1
        # self.Ro = 11
        # self.Ra = 26
		self.max_omega = 70*np.pi/180
		self.visibility_range = 120*np.pi/180
        # self.max_omega = 100*np.pi/180
        # self.visibility_range = 150*np.pi/180
		self.agent_body_size = 0.1
		self.goal_threshold_radius = 5
		self.num_Obs = 0
		self.Obstacles = []
		self.min_Obs_size = 1
		self.max_Obs_size = 6
		for i in range(self.num_Obs):
			self.Obstacles.append([np.random.uniform(self.boundary_min,self.boundary_max,2),
                                    np.random.uniform(self.min_Obs_size,self.max_Obs_size),
                                    np.random.uniform(self.boundary_min,self.boundary_max,2)])
		self.goal_pose = np.random.uniform(self.boundary_min+self.goal_threshold_radius,
                                            self.boundary_max-self.goal_threshold_radius,2)         # Goal can be made dynamic for goal chasing problem
		self.agent_speed = 5
		self.leader_speed = 3
		self.rotation_act = 70*np.pi/180 # its Omega
    

class Swarm:
	def __init__(self):
		self.P = params()

	def agentGenerator(self,num,obj,a_type):           # For heterogenous agents
		agents = []
		for i in range(num):
			agents.append(obj(a_type,i,self.P))
		return agents
    
	def saturate(self,val,lim):
		if abs(val)>lim:
			val = lim*(val/abs(val))
		return val
    
	def neighbours(self,thisAgent,allAgents):
		for agent in allAgents:
			if agent.id != thisAgent.id:
				d = np.linalg.norm(agent.pose - thisAgent.pose)
			relative_bearing = self.blindRegion(thisAgent,agent)
			if d<=self.P.Rr and relative_bearing<=self.P.visibility_range:
				thisAgent.zor.append(agent)
			elif d>self.P.Rr and d<=self.P.Ro and relative_bearing<=self.P.visibility_range:
				thisAgent.zoo.append(agent)
			elif d>self.P.Ro and d<=self.P.Ra and relative_bearing<=self.P.visibility_range:
				thisAgent.zoa.append(agent)
			else:
				pass

	def blindRegion(self,thisAgent,otherAgent):
		pointing_vec = otherAgent.pose - thisAgent.pose
		norm = np.linalg.norm(pointing_vec)
		vel = np.linalg.norm(thisAgent.velocity)
		pointing_vec = pointing_vec/norm
		if vel!=0:
			dot = np.dot(pointing_vec,thisAgent.velocity)/vel
		else:
			dot = 0

		relative_bearing = np.arccos(dot)
		return relative_bearing

	def repulsion(self,thisAgent):
		repel = np.zeros(2)
		for i in thisAgent.zor:
			vec = i.pose - thisAgent.pose
			vec_n = np.linalg.norm(vec)
		if vec_n != 0:
			repel = repel + vec/vec_n
		repulsion = -repel/len(thisAgent.zor)
		return repulsion
    
	def alignment(self,thisAgent):
		align = np.zeros(2)
		for i in thisAgent.zoo:
			vel_n = np.linalg.norm(i.velocity)
			if vel_n != 0:
				align = align + i.velocity/vel_n
		alignment = align/len(thisAgent.zoo)
		return alignment
    
	def cohesion(self,thisAgent):
		attract = np.zeros(2)
		for i in thisAgent.zoa:
			vec = i.pose - thisAgent.pose
			vec_n = np.linalg.norm(vec)
			if vec_n != 0:
				attract = attract + vec/vec_n
		attraction = attract/len(thisAgent.zoa)
		return attraction

	def steer(self,thisAgent,action):
		if thisAgent.a_type=='SA':
			if len(thisAgent.zor) != 0:
				net_steer = self.repulsion(thisAgent)
			else:
				if len(thisAgent.zoo) != 0 and len(thisAgent.zoa) == 0:
					net_steer = self.alignment(thisAgent)
				elif len(thisAgent.zoo) == 0 and len(thisAgent.zoa) != 0:
					net_steer = self.cohesion(thisAgent)
				elif len(thisAgent.zoo) != 0 and len(thisAgent.zoa) != 0:
					net_steer = 0.5*(self.alignment(thisAgent)+self.cohesion(thisAgent))
				else:
					vel_norm = np.linalg.norm(thisAgent.velocity)
					if vel_norm!=0:
						net_steer = thisAgent.velocity/vel_norm
					else:
						net_steer = thisAgent.velocity

		else:
			if len(thisAgent.zor) != 0:
				net_steer = self.repulsion(thisAgent)
			else:       
				net_steer = np.subtract(self.P.goal_pose,thisAgent.pose)

		return net_steer

	def angleWrap(self,angle):
		'''0 to 2Pi'''
		while angle<0:
			angle += 2*np.pi
		while angle > 2*np.pi:
			angle-=2*np.pi
		return angle

	def angleWrap1(self,angle):
		'''-Pi to Pi'''
		while angle<-np.pi:
			angle += 2*np.pi
		while angle > np.pi:
			angle-=2*np.pi
		return angle

	def mean_agent(self,agents):
		pose = np.zeros(2)
		vel = np.zeros(2)
		compass = 0
		for a in agents:
			pose += a.pose
			compass += a.heading
			vel += a.velocity
		pose /= len(agents)
		compass /= len(agents)
		vel /= len(agents)
		return np.array([pose[0],pose[1],compass,vel[0],vel[1]]) #[x,y,theta,vx,vy]

class Agent:
	def __init__(self,a_type,id,P):
		self.P = P
		self.a_type = a_type
		self.id = id
		self.pose = np.random.uniform(0,self.P.init_patch_radius,2) + self.P.init_patch_center
		if self.P.num_Obs!=0:
			d = []
			for i in self.P.Obstacles:
				d.append(np.linalg.norm(self.pose-i[0])-i[1])
			d[d<=0] = 0
			d[d>0] = 1
			while 0 in d:
				self.pose = np.random.uniform(0,self.P.init_patch_radius,2) + self.P.init_patch_center
				d = []
				for i in self.P.Obstacles:
					d.append(np.linalg.norm(self.pose-i[0])-i[1])
				d[d<=0] = 0
				d[d>0] = 1    
		if self.a_type=="L":
			self.heading = np.random.uniform(0,2*np.pi)#(0,2*np.pi)#
		else:
			self.heading = np.random.uniform(0,2*np.pi)#(np.pi/6,np.pi/3)#(0,2*np.pi)#np.random.uniform

		self.velocity = self.P.agent_speed*np.array([np.cos(self.heading),np.sin(self.heading)])
		self.zoa = []
		self.zoo = []
		self.zor = []
		self.Rr = self.P.Rr
		self.Ro = self.P.Ro
		self.Ra = self.P.Ra
		self.visibility_range = self.P.visibility_range
		self.interaction_force = None
		self.obstacle_force = None
		self.remaining_path = None
		self.ang_vel = 0
		if self.a_type=="L":
			self.action_space = [self.P.rotation_act,0,-self.P.rotation_act]
			self.n_actions = len(self.action_space)
			self.n_states = None
			self.choosen_action = None
			self.Q_eval = None
			self.Q_next = None
			self.epsilon = self.P.epsilon
			self.learn_call_counter = 0
			self.memory = None
            

goal_direction = None

P = params()
World = Swarm()
SA_agents = World.agentGenerator(P.num_SA,Agent,P.a_type)


class Publisher(Node):
	def __init__(self):
		super().__init__('publisher_subscriber')
		self.publisher_ = None
		time_period = 0.1
		self.timer = self.create_timer(time_period,self.loop_callback)
	
	def callback(self,msg,thisAgent):
		thisAgent.pose = np.array([msg.x,msg.y])
		thisAgent.heading = msg.theta
		World.neighbours(thisAgent,SA_agents)

		steering = (World.steer(thisAgent,0))
		if np.linalg.norm(steering)!=0:
		      steering = steering/np.linalg.norm(steering)
		

		angular_vel = World.angleWrap1(World.angleWrap(np.arctan2(steering[1],steering[0])) - World.angleWrap(thisAgent.heading))
		thisAgent.heading += World.saturate(angular_vel,P.dt*P.max_omega)
		thisAgent.velocity = np.array([np.cos(thisAgent.heading),np.sin(thisAgent.heading)])*P.agent_speed
		thisAgent.pose += P.dt*thisAgent.velocity
		thisAgent.ang_vel = angular_vel
		thisAgent.zor = []
		thisAgent.zoo = []
		thisAgent.zoa = []
	
	def loop_callback(self):
		for i in range(len(SA_agents)):
			laser_scans = self.create_subscription(Pose2D,'/tb3_'+str(i)+'/scan',partial(self.callback,SA_agents[i]),10)
			velocity_message = Twist()
			cmd_vel_topic='/tb3_'+str(i)+'/cmd_vel'
			self.publisher_ = self.create_publisher(Twist,cmd_vel_topic,10)
			velocity_message.linear.x = np.float(P.agent_speed)
			velocity_message.angular.z = np.float(SA_agents[i].ang_vel)
			self.publisher_.publish(velocity_message)

	
rclpy.init()
pub = Publisher()
rclpy.spin(pub)
pub.destroy_node()
rclpy.shutdown()

