#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from enum import Enum

LIN_VEL_STEP_SIZE = 0.1
class Direction(Enum):
    FORWARD = 1
    BACKWARD = 2


class TurtleBot3PathMover(Node):
    def __init__(self):
        super().__init__('turtlebot3_path_mover')
        
        # Parameters (can be made configurable later)
        self.declare_parameter('path_length', 3.5)  # path length in meters
        self.declare_parameter('linear_speed', 0.6)  # speed in m/s
        
        self.path_length = self.get_parameter('path_length').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.speed = 0.0
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'dummy/cmd_vel', 10)
        
        # Subscriber for odometry
        self.odom_sub = self.create_subscription(Odometry,'dummy/odom',self.odom_callback,10)
            
        # State variables
        self.current_position = (0.0, 0.0)
        self.start_position = (0.0, 0.0)
        self.direction = Direction.FORWARD
        self.is_first_odom = True
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        # self.get_logger().info(f'TurtleBot3 Path Mover started with path_length={self.path_length}m, speed={self.linear_speed}m/s')
        
    def odom_callback(self, msg:Odometry):
        # Extract position from odometry message
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Initialize start position on first odometry message
        if self.is_first_odom:
            self.start_position = self.current_position
            self.is_first_odom = False
            self.get_logger().info(f'Initial position: ({self.start_position[0]:.2f}, {self.start_position[1]:.2f})')
        
    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        
    def control_loop(self):
        # Skip if we haven't received odometry yet
        if self.is_first_odom:
            return
            
        # Calculate distance traveled from start position
        distance_traveled = self.calculate_distance(self.start_position, self.current_position)
        
        # Check if we need to change direction
        
        if distance_traveled >= self.path_length:
            # Switch direction
            if self.direction == Direction.FORWARD:
                self.direction = Direction.BACKWARD
                # self.get_logger().info(f'Switching to BACKWARD direction after traveling {distance_traveled:.2f}m')
            else:
                self.direction = Direction.FORWARD
                # self.get_logger().info(f'Switching to FORWARD direction after traveling {distance_traveled:.2f}m')
                
            # Update start position for next segment
            
            self.start_position = self.current_position
        
        # Create and publish velocity command based on direction
        twist_msg = Twist()
        
        if self.direction == Direction.FORWARD:
            self.target_speed = self.linear_speed 
        else:
            self.target_speed = -self.linear_speed 

        self.speed = make_simple_profile(self.speed, self.target_speed, LIN_VEL_STEP_SIZE/2)    
        twist_msg.linear.x = self.speed

        self.cmd_vel_pub.publish(twist_msg)
        
    def stop_robot(self):
        # Publish zero velocity to stop the robot
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info('Robot stopped')

def make_simple_profile(output, input, slop):
    if input - output > 0.005:
        output = output + slop
    elif input - output < -0.005:
        output = output - slop
    else:
        output = input

    return output

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3PathMover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by keyboard interrupt')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()