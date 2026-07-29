#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths to necessary resources
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    urdf = os.path.join(pkg_turtlebot3_gazebo, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    # World file path
    world_path = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )
    # Robot spawning configurations
    robots = [
        {
            'name': 'robot1',
            'x_pos': 0.0,
            'y_pos': 0.0,
            'z_pos': 0.0,
            'namespace': 'robot1'
        },
        {
            'name': 'robot2',
            'x_pos': 2.0,
            'y_pos': 2.0,
            'z_pos': 0.0,
            'namespace': 'robot2'
        }
    ]
    ld = LaunchDescription()
    # Nodes for spawning robots

    urdf_path = os.path.join(
        pkg_turtlebot3_gazebo, 
        'models', 
        'turtlebot3_burger',
        'model.sdf'
    )
    for robot in robots:

        # # Spawn robot node
        ld.add_action(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot['name'],
                '-file', urdf_path,
                '-x', str(robot['x_pos']),
                '-y', str(robot['y_pos']),
                '-z', str(robot['z_pos']),
                # '-topic', f'/{robot["namespace"]}/robot_description',
                '-robot_namespace', robot['namespace']
            ],
            output='screen'
        )
        )
        # Robot state publisher
        ld.add_action( Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot['namespace'],
            parameters=[{
                'robot_description': robot_description
            }]
        )
        )

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='test',
        arguments=[
            '-entity', 'burger',
            '-file', urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace','robot1',
        ],
        output='screen',
            
            
        )


    # Combine all launch actions
    ld.add_action(gazebo)

    # ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld