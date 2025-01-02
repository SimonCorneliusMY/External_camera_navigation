from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),

        
        # Launch usb_cam node with parameters
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'pixel_format':'raw_mjpeg',
                'framerate': 30.0,
                'image_width': 1280,
                'image_height': 720
            }],
            output='screen'
        ),
    ])

