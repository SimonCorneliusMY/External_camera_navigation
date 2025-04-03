import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Camera addresses
    sim_0 = [[502,202,1076,226,171,531,1278,640],[0,0,1112,0,0,1163,1112,1163]]
    sim_1 = [[196,115,730,26,286,718,1220,424],[0,0,980,0,0,1236,980,1236]]
    sim_0_Large = [[636,73,1000, 77,171,531,1278,640],[0,0,1112,0,2855,0,1112,2855]]
    real_0 = [[598,134,903,169,163,457,1079,619],[0,0,1203,0,0,2869,1203,2869]]
    res_real_0 = 3.57/930   #3.54/1203
    res_sim_0 = 3.45787/1112
    res_sim_1 = 3.45787/979
    resolutions = [res_sim_0,res_sim_1]
    camera_addresses = ["0","1"]
    homographic_ori_points = [sim_0[0],sim_1[0]]
    homographic_transformed_points = [sim_0[1],sim_1[1]]
    tf_sim_0 = ["0", "3.633", "0", "3.14159", "0", "0"]
    tf_sim_1 = ["3.45787", "-3.633", "0", "0", "3.14159", "0"]
    tf_sim_0_large = ["0", "7.99", "0", "3.14159", "0", "0"]
    tf_real_0 = ["0", "8.24", "0", "3.14159", "0", "0"]
    tf = [tf_sim_0, tf_sim_1]
    # Initialize the LaunchConfiguration object for 'use_sim_time'
    use_sim_time = LaunchConfiguration('use_sim_time')


    # Declare the use_sim_time argument, with a default value of 'true'
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)


    # Loop through each camera address to add relevant nodes
    i = 0
    for camera_address in camera_addresses:
        # Static tf publisher option 1, use tf2_ros package, this creates a new node for each static tf, option 2, write the static tf in camera
        ld.add_action(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = ['--x', tf[i][0], '--y', tf[i][1], '--z', tf[i][2], '--roll', tf[i][3], '--pitch', tf[i][4], '--yaw', tf[i][5], '--frame-id', 'map', '--child-frame-id', f"map_{i}"]
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
                            {'use_sim_time': use_sim_time},
                            {'resolution': resolutions[i]},
                            {'homographic_ori_points': homographic_ori_points[i]},
                            {'homographic_transformed_points': homographic_transformed_points[i]}]
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
                            {'use_sim_time': use_sim_time}],
                # condition = UnlessCondition(use_sim_time)
            )
        )

        # Increment the camera index
        i += 1



    # Add the declared use_sim_time argument to the launch description
    # ld.add_action(declare_use_sim_time_cmd)

    # Add all the nodes to the launch description


    return ld
