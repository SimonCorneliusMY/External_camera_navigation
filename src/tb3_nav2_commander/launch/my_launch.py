# example_launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile

def generate_launch_description():

    robot_localization_dir = get_package_share_directory('robot_localization')




    # args that can be set from the command line or a default will be used
    bringup_dir = get_package_share_directory('nav2_bringup')
    map_yaml = DeclareLaunchArgument(
        "map", description='Full path to map yaml file to load'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')


    # include a Python launch file in the chatter_py_ns namespace
    launch_py_include_with_namespace = GroupAction(
        actions=[
            # push_ros_namespace to set namespace of included nodes
            PushRosNamespace('my_space'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('robot_localization'),
                        'launch/ekf.launch.py'))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('turtlebot3_gazebo'),'launch/my_world.launch.py')),

            ),

              
        ]
    )
    img_sub_gazebo_node = Node(
        package='my_localizer',
        executable='my_localizer',
        name=LaunchConfiguration('my_localizer'),
        
    )


    # start a turtlesim_node in the turtlesim1 namespace
    turtlesim_node = Node(
        package='turtlesim',
        namespace='turtlesim1',
        executable='turtlesim_node',
        name='sim'
    )

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    turtlesim_node_with_parameters = Node(
        package='turtlesim',
        namespace='turtlesim2',
        executable='turtlesim_node',
        name='sim',
        parameters=[{
            "background_r": LaunchConfiguration('background_r'),
            "background_g": LaunchConfiguration('background_g'),
            "background_b": LaunchConfiguration('background_b'),
        }]
    )

    # perform remap so both turtles listen to the same command topic
    forward_turtlesim_commands_to_second_turtlesim_node = Node(
        package='turtlesim',
        executable='mimic',
        name='mimic',
        remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ]
    )

    return LaunchDescription([
        img_sub_gazebo_node,
        launch_py_include_with_namespace,
        name,
        

    ])