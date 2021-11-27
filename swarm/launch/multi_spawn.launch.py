#!/usr/bin/env python3

import sys
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def agents_list(num_of_agents):

    agents = []

    for i in range(num_of_agents):
        agent_name = "tb3_"+str(i)
        x = np.random.uniform(0,50)
        y = np.random.uniform(0,50)                         
        w = np.random.uniform(0,360)
        agents.append({'name': agent_name, 'x_pose': x, 'y_pose': y, 'z_pose': 0.01,'w_pose':w})

    return agents 

def generate_launch_description():

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    sdf_file = 'turtlebot3_' + TURTLEBOT3_MODEL +'/model.sdf'
    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)
    sdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        sdf_file)
    # Names and poses of the robots
    robots = agents_list(50)

    # We create the list of spawn robots commands
    spawn_robots_cmds = []
    for robot in robots:

        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('swarm'),
                                                           'spawn_tb3.launch.py')),
                launch_arguments={
                                  'robot_urdf':sdf,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'w': TextSubstitution(text=str(robot['w_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_namespace': robot['name']
                                  }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()
    
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld
