# Copyright (c) 2018 Intel Corporation
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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node



def generate_launch_description():

    sim_0 = [[502,202,1076,226,171,531,1278,640],[0,0,1112,0,0,1163,1112,1163]]
    sim_0_low_res = [[502,202,1076,226,171,531,1278,640],[0,0,541,0,0,568,541,568]]
    sim_1_low_res = [[196,115,730,26,286,718,1220,424],[0,0,541,0,0,682,541,682]]
    sim_0_high_res = [[484,185,1034,205,115,510,1210,618],[0,0,550,0,0,618,550,618]]
    sim_1_high_res = [[392,80,1000,78,13,542,1275,585],[0,0,550,0,0,618,550,618]]
    sim_0_Large = [[636,73,1000, 77,171,531,1278,640],[0,0,1112,0,2855,0,1112,2855]]
    real_0 = [[598,134,903,169,163,457,1079,619],[0,0,1203,0,0,2869,1203,2869]]

    res_real_0 = 3.57/930   #3.54/1203
    res_sim_0 = 3.45787/1112
    res_sim_1 = 3.56/550
    low_res_0 = 3.45787/575.00
    low_res_1 = 3.45787/541.00

    tf_sim_0 = ["0", "4.0", "0", "3.14159", "0", "0"]
    tf_sim_1 = ["3.56", "4.0", "0", "0", "3.14159", "0"]
    tf_sim_0_large = ["0", "7.99", "0", "3.14159", "0", "0"]
    tf_real_0 = ["0", "8.24", "0", "3.14159", "0", "0"]

    tf = [tf_sim_0, tf_sim_1]
    resolutions = [res_sim_1,res_sim_1]
    camera_addresses = ["0","1"]
    homographic_ori_points = [sim_0_high_res[0],sim_1_high_res[0]]
    homographic_transformed_points = [sim_0_high_res[1],sim_1_high_res[1]]
    # Get the launch directory
    bringup_dir = get_package_share_directory('tb3_nav2_commander')
    launch_dir = os.path.join(bringup_dir, 'launch')


    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        # default_value=os.path.join(bringup_dir, 'params', 'nav2_params_sim_241205.yaml'),
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params_sim_MPPI_250327.yaml'),
        # default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'my_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        # worlds/turtlebot3_worlds/waffle.model')
        default_value=os.path.join(bringup_dir, 'worlds', 'TB3_maze_simulation_v3.world'),
        description='Full path to world model file to load')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_burger',
        description='name of the robot')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(bringup_dir, 'worlds', 'burger.model'),
        description='Full path to robot sdf file to spawn the robot in gazebo')


    #my nodes
    # Loop through each camera address to add relevant nodes
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)





    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'map', '--child-frame-id', 'odom']
        )

    pose_aggregator = Node(
        package='my_localizer',
        executable='pose_aggregator',
        name='pose_aggregator',
        output = 'screen',
        parameters = [{'robot_name': 'TurtleBot3'},
                     {'num_cameras': 2},
                     {'use_sim_time': use_sim_time}]
        )
    
    map_merger = Node(
        package='mapping_cpp',
        executable='map_merger',
        name='map_merger',
        output = 'screen',
        parameters= [{'resolution': resolutions[0]},
                     {'camera_addresses': camera_addresses},
                     {'use_sim_time': use_sim_time}]
    )



    gazebo_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'my_world.launch.py')),
        
        )


    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'my_bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())

    # Create the launch description and populate
    i = 0

    # You will need to launch target_pose node on your own as it is not include in the launch file
    # This node merely detects the target, in this case fire hydrant
    target_pose = Node(
        package='my_localizer',
        executable='localizer_real',  # Replace with your executable name
        name=f'localizer_real_target',  # Unique name for each instance
        output='screen',
        parameters=[{'name': '0'},
                    {'show_homographic_region': True},
                    {'record': False},
                    {'publish_pose_tf': False},
                    {'use_sim_time': use_sim_time},
                    {'resolution': resolutions[i]},
                    {'homographic_ori_points': homographic_ori_points[i]},
                    {'homographic_transformed_points': homographic_transformed_points[i]},
                    {'yolo_model_path': "/home/tarumt2204/YOLOv8_ws/yolov8n.pt"}]
    )

    for camera_address in camera_addresses:
        # Static tf publisher option 1, use tf2_ros package, this creates a new node for each static tf, option 2, write the static tf in camera
        ld.add_action(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = ['--x', tf[i][0], '--y', tf[i][1], '--z', tf[i][2], '--roll', tf[i][3], '--pitch', tf[i][4], '--yaw', tf[i][5], '--frame-id', 'map', '--child-frame-id', f"map_{i}"],
                parameters= [{'use_sim_time' : use_sim_time}]
            )
        )

        # Only launch camera node if use_sim_time is False (simulated time is not used)
        ld.add_action(
            Node(
                package='camera_cpp',
                executable='camera_publisher',  # Replace with your executable name
                name=f'camera_node_{i}',  # Unique name for each instance
                output='screen',
                parameters=[{'camera_address': camera_address},
                            {'name': f'{i}'},
                            {'view_feed': False}],
                condition = UnlessCondition(use_sim_time)
            )
        )
    
        # Localizer node
        ld.add_action(
            Node(
                package='my_localizer',
                executable='localizer_real',  # Replace with your executable name
                name=f'localizer_real_{i}',  # Unique name for each instance
                output='screen',
                parameters=[{'name': f'{i}'},
                            {'show_homographic_region': True},
                            {'record': False},
                            {'publish_pose_tf': False},
                            {'use_sim_time': use_sim_time},
                            {'resolution': resolutions[i]},
                            {'homographic_ori_points': homographic_ori_points[i]},
                            {'homographic_transformed_points': homographic_transformed_points[i]},
                            # {'yolo_model_path': "/home/tarumt2204/YOLOv8_ws/runs/detect/TB3_sim_251103/weights/best.pt"}]
                            {'yolo_model_path': "/home/tarumt2204/YOLOv8_ws/runs/detect/2880/weights/best.pt"}]
            )
        )
        
        # Mapping node
        ld.add_action(
            Node(
                package='mapping_cpp',
                executable='mapping_real_cpp',  # Replace with your executable name
                name=f'mapping_{i}',  # Unique name for each instance
                output='screen',
                parameters=[{'name': f'{i}'},
                            {'save_map': False},
                            {'show_fps': False},
                            {'use_sim_time': use_sim_time},
                            {'HSV': [10, 0, 0, 30, 255, 255]},
                            {'inflation': 10},
                            {'age_penalty': 0.4},
                            {'resolution': resolutions[i]},
                            {'homographic_ori_points': homographic_ori_points[i]},
                            {'homographic_transformed_points': homographic_transformed_points[i]}],
                # condition = UnlessCondition(use_sim_time)
            )
        )

        # Increment the camera index
        i += 1

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    # ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add any conditioned actions

    ld.add_action(map_to_odom_tf)

    

    # Add the actions to launch all of the navigation nodes

    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(gazebo_turtlebot_cmd)
    ld.add_action(pose_aggregator)
    ld.add_action(map_merger)
    # ld.add_action(target_pose)


    return ld
