#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Path


class CmdVelVisualizer(Node):
    def __init__(self):
        super().__init__('cmd_vel_visualizer')
        
        # QoS profile to ensure reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability =DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribe to the cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        self.wheel_joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.tb3_cmd_vel_input_sub = self.create_subscription(Twist, 'cmd_vel_input',self.cmd_vel_input_callback, 10)

        # Publisher for visualization markers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'cmd_vel_markers',
            qos_profile
        )

        self.direction = 0.0
        self.scale1 = 0.5
        self.scale2 = 1.0
        self.scale3 = 1.5
        self.base_pose_z = 0.3
        self.base_pose_increase = 0.02
        self.diameter_increase = 0.2
        self.diameter = 0.5
        
        self.get_logger().info('CMD_VEL Visualizer has started. Listening to cmd_vel topic...')

    def cmd_vel_input_callback(self, msg:Twist):
        self.cmd_vel_input = msg

    def joint_state_callback(self,msg:JointState):
        self.direction = msg.velocity[0] - msg.velocity[1]



    def cmd_vel_callback(self, msg:Twist):
        # Create markers to visualize the cmd_vel
        marker_array = MarkerArray()
        # delay = 500e-9
        # time = Time()
        # time.sec = self.get_clock().now().seconds_nanoseconds
        # time =self.get_clock().now().nanoseconds + delay
        
        # Create a marker for the linear velocity (an arrow)
        linear_marker = Marker()
        linear_marker.header.frame_id = "base_link"  # Reference frame
        linear_marker.header.stamp = self.get_clock().now().to_msg()
        linear_marker.ns = "cmd_vel_visualization"
        linear_marker.id = 0
        linear_marker.type = Marker.ARROW
        linear_marker.action = Marker.ADD

        # Start point of the arrow
        linear_marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        
        # End point of the arrow - based on linear velocity
        # Scale the arrow length for better visualization
        scale_factor = 1.0
        linear_marker.points.append(Point(
            x=msg.linear.x * scale_factor,
            y=msg.linear.y * scale_factor,
            z=msg.linear.z * scale_factor
        ))
        
        # Set the arrow size
        linear_marker.scale.x = 0.05  # shaft diameter
        linear_marker.scale.y = 0.1   # head diameter
        linear_marker.scale.z = 0.1   # head length
        linear_marker.pose.orientation
        
        # Set color (red)
        linear_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        # Add to marker array
        marker_array.markers.append(linear_marker)
        
        # Create a marker for the angular velocity (a spinning disk)
        angular_marker = Marker()
        angular_marker.header.frame_id = "base_link"
        angular_marker.header.stamp = self.get_clock().now().to_msg()
        angular_marker.ns = "cmd_vel_visualization"
        angular_marker.id = 1
        angular_marker.type = Marker.CYLINDER
        angular_marker.action = Marker.ADD
        
        # Set position and size
        angular_marker.pose.position.x = 0.0
        angular_marker.pose.position.y = 0.0
        angular_marker.pose.position.z = self.base_pose_z  # slightly above the ground
        
        # Calculate quaternion for rotation based on angular velocity
        angular_speed = abs(msg.angular.z)
        angular_marker.scale.x = self.diameter # diameter
        angular_marker.scale.y = self.diameter # diameter
        angular_marker.scale.z = 0.01  # height
        
        # Set color (blue) with alpha based on angular speed
        alpha = min(1.0, angular_speed)
        if msg.angular.z > 0:
            # Counter-clockwise rotation (blue)
            angular_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=max(0.3, alpha))
        else:
            # Clockwise rotation (green)
            angular_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=max(0.3, alpha))
        
        # Add to marker array
        marker_array.markers.append(angular_marker)


        angular_marker_2 = Marker()

        angular_marker_2.header.frame_id = "base_link"
        angular_marker_2.header.stamp = self.get_clock().now().to_msg()
        angular_marker_2.ns = "cmd_vel_visualization"
        angular_marker_2.id = 2
        angular_marker_2.type = Marker.CYLINDER
        angular_marker_2.action = Marker.ADD

        # Set position and size
        angular_marker_2.pose.position.x = 0.0
        angular_marker_2.pose.position.y = 0.0
        angular_marker_2.pose.position.z = self.base_pose_z - self.base_pose_increase  # slightly above the ground
        
        angular_marker_2.scale.x = self.diameter + self.diameter_increase # diameter
        angular_marker_2.scale.y = self.diameter + self.diameter_increase  # diameter
        angular_marker_2.scale.z = 0.01  # height

        alpha = min(1.0, abs(self.direction))
        if self.direction > 0:
            # Counter-clockwise rotation (blue)
            angular_marker_2.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=max(0.3, alpha))
        else:
            # Clockwise rotation (green)
            angular_marker_2.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=max(0.3, alpha))

        marker_array.markers.append(angular_marker_2)
        
        #Delay, for some reason the marker node time is faster than nav2 time, i guess nav2 is waiting on certain tasks and publishes delayed time.
        time.sleep(0.05)

        # Publish the marker array
        self.marker_publisher.publish(marker_array)
        
        # Log the received cmd_vel for debugging
        self.get_logger().info(
            f'Received cmd_vel - Linear: [{msg.linear.x}, {msg.linear.y}, {msg.linear.z}], '
            f'Angular: [{msg.angular.x}, {msg.angular.y}, {msg.angular.z}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()