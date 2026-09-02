#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool


# Adapted: Simon Peter Cornelius
'''
6/8/26
Launches SE008 room with turtlebot3
Used in my_multi_tb3_simulation_launch
'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='3.046540')
    y_pose = LaunchConfiguration('y_pose', default='12.616602')
    urdf = os.path.join(pkg_turtlebot3_gazebo, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    urdf_path = os.path.join(
        pkg_turtlebot3_gazebo, 
        'models', 
        'turtlebot3_burger',
        'model.sdf')

    tb3_dir = get_package_share_directory('tb3_nav2_commander')
    world = os.path.join(
        tb3_dir,
        'worlds',
        'SE_008.world'
    )
    
    models = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(tb3_dir, 'models')
    )

    

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
        'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
        # # Spawn robot node
    dummy_tb3_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'dummy_tb3',
            '-file', urdf_path,
            # '-x', '2.0',
            # '-y', '10.0',
            '-x', '1.605339',
            '-y', '10.042339',
            '-z', '0.0',
            '-robot_namespace', 'dummy'
        ],
        output='screen'
        )
        
        # Robot state publisher
    dummy_tb3_robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='dummy',
        parameters=[{
            'robot_description': robot_description
        }]
    )
        




    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(models)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    
    # ld.add_action(dummy_tb3_spawn)        # uncomment to spawn another turtlebot3
    # ld.add_action(dummy_tb3_robot_state)  # uncomment to spawn another turtlebot3
    # ld.add_action(my_turtlebot3_drive)


    return ld
