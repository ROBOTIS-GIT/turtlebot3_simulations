# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    turtlebot3_ignition_path = get_package_share_directory('turtlebot3_ignition')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    control_params_file = PathJoinSubstitution(
        [turtlebot3_ignition_path,
         'config', 'turtlebot3_' + TURTLEBOT3_MODEL + '.yaml'])

    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    load_diff_drive_base_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['diff_drive_base_controller', '--controller-manager', '/controller_manager'],
        parameters=[control_params_file, {'use_sim_time': use_sim_time}],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # Ensure diffdrive_controller_node starts after load_joint_state_controller
    diff_drive_base_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_diff_drive_base_controller],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        load_joint_state_controller,
        diff_drive_base_controller_callback
    ])
