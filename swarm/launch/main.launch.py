#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
	use_sim_time = LaunchConfiguration('use_sim_time', default='True')
	
	launch_file_dir = get_package_share_directory('swarm')
	pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
	gzserver = IncludeLaunchDescription(PythonLaunchDescriptionSource(
	os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),)
    
	gzclient = IncludeLaunchDescription(PythonLaunchDescriptionSource(
	os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),)
	
	execute_pro = ExecuteProcess(cmd=['ros2', 'param', 'set', '/gazebo', 
	'use_sim_time', use_sim_time],output='screen')
            
	multi_spawn = IncludeLaunchDescription(PythonLaunchDescriptionSource(
	os.path.join(launch_file_dir,'multi_spawn.launch.py')),)
	
	
	return LaunchDescription([gzserver,
	gzclient,
	execute_pro,
	multi_spawn,])
