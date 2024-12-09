#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim

import os
import select
import sys
import rclpy
import cv2 as cv
from rclpy.action import ActionServer
from rclpy.node import Node
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu #SPC
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from turtlebot3_msgs.msg import Astar       #SPC created
from turtlebot3_msgs.srv import Astarservice    #SPC created
#from turtlebot3_msgs.srv import astarservice

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22m/s, Waffle and Waffle Pi : ~ 0.26m/s)
a/d : increase/decrease angular velocity (Burger : ~ 2.84rad/s, Waffle and Waffle Pi : ~ 1.82rad/s)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


class teleop(Node):
    def __init__(self):
        super().__init__('teleop')

        #create subscriptions to topics (i get info from topics)
        self.odom_subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.motion_subscription = self.create_subscription(Astar,'motioncontrol',self.motion_callback,10)
        self.location_subscription = self.create_subscription(Astar,'location',self.location_callback,10)
        self.tf_subscription = self.create_subscription(TFMessage,'tf',self.tf_callback,10)
        #self.actionserver = ActionServer()

        #self.create_client(Astarservice,'/astar',10,self.motion_callback) #14/6/2024 probably wont work, creating a client service connection between astarservice node and teleop node
        #create publisher to topics (i give info to topics)
        self.teleop_publisher = self.create_publisher(Twist,'cmd_vel',10)   

        #attributes (I have no idea what some of them are for)
        self.astar_path2  = ['angle','magnitude','coordinate']   #20/6/2024 maybe use dictionary type to make it more readable, cons, more memory usage
        self.astar_path = {'angle':[],'magnitude':[],'coordinate':[]}
        self.pose_data = ['x','y']
        self.location = {'turtlebot3_location': [np.array([0,0])],'goal_location':[]}
        self.past_angle = 0
        self.current_angle = 0
        self.target_angle = 0
        self.factor = 1
        self.orientation_z = 0
        self.orientation_z_deg = 0
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.xs = []
        self.ys = []

    #callback functions required for subscription to work, I save the information to some attribute to use later, raw data may be transform for usage like imu data
    def tf_callback(self,data):
        #print(len(data.transforms))
        return
    
    
    def location_callback(self,data):
        distance_error = 0.2  #arbitrary value
        if np.sqrt(np.sum((self.location['turtlebot3_location'][-1] - data.coordinate[0:2])**2,)) > distance_error: #store location when TurtleBot3 a distance greater than distance error
        #if np.any(self.location['turtlebot3_location'][-1] != data.coordinate[0:2]):    #avoid storing the same location value
            self.location['turtlebot3_location'].append(np.array(data.coordinate[0:2]))  #appends the location of turtlebot3

        self.location['turtlebot3_location'] = self.location['turtlebot3_location'][-5:] #only keep the latest/last 5 values
        self.location['goal_location'] = data.coordinate[2:5]   #slice the list taking the 3rd and 4th value
        
        #25/6/2024 to be placed in a function, to obtain the TurtleBot3 angular heading from it motion through the camera
        #26/6/2024 this is method is impractical as to obtain a sensitivity of 1deg a distance of 45 pixels is needed, pixels=45/degree

        # if len(self.location['turtlebot3_location'])>1:
        #     #print(self.location['turtlebot3_location'])
        #     rc = self.location['turtlebot3_location'][-1] - self.location['turtlebot3_location'][0]
        # print(math.atan2(-rc[0],rc[1])*180/math.pi, rc, self.location['turtlebot3_location'][-1] ,self.location['turtlebot3_location'][-2]) 
        # orientation = math.atan2(-rc[0],rc[1])*180/math.pi  #-rc[0] is due row axis is the negative y axis, hence to invert it back.

    
    def odom_callback(self,data):
        self.pose_data[0] = data.pose.pose.position.x
        self.pose_data[1] = data.pose.pose.position.y
        

    def motion_callback(self,data):     #18/6/2024 cheat method of communicating between astarservice and teleop by using topic instead of service

        self.astar_path['angle']=data.angle
        self.astar_path['magnitude']=[data.magnitude,0]   #to make the magnitude and angle array the same length
        self.astar_path['coordinate']=np.reshape(data.coordinate,(int(len(data.coordinate)/2),2)) 



    def imu_callback(self,msg):
        #conversion code, convert the imu sensor data from triangle waveform to saw-tooth waveform when plotting imu data vs orientation
        q0 = msg.orientation.w
        q3 = msg.orientation.z
        angle_rad = math.atan2(2*q0*q3,q0**2-q3**2)
        self.orientation_z_deg = ((angle_rad+2*math.pi)%(2*math.pi))*180/math.pi
        #print(angle_rad,self.orientation_z_deg)
        

        #1/7/2024, old code, to be deleted if not used
        # if (((round(msg.orientation.z,5)-self.past_angle < 0 ) and (msg.angular_velocity.z>0.02))
        #     or ((round(msg.orientation.z,5)-self.past_angle > 0 ) and (msg.angular_velocity.z<-0.02))):
        #     self.factor = -1
        #     #print('+',(round(msg.orientation.z,3)-self.past_angle > 0),(round(msg.orientation.z,3)-self.past_angle < 0),msg.angular_velocity.z)
        # elif (msg.angular_velocity.z>0.05 or msg.angular_velocity.z<-0.05):
        #     self.factor = 1
        #     #print("-")
        # delta_angle = abs(abs(self.past_angle) - abs(msg.orientation.z))*180
        # self.current_angle = self.current_angle + delta_angle*np.sign(np.mod(self.target_angle/180- self.orientation_z+3,2)-1)
        # self.past_angle = msg.orientation.z
        # self.orientation_z = msg.orientation.z*self.factor
        #1/7/2024, old code, to be deleted if not used

        #plotting code
        # anim = animate(0,self.orientation_z,self.xs,self.ax)
        # ani = animation.FuncAnimation(self.fig, anim, interval=1000)
        # self.xs.append(self.orientation_z)
        # self.ys.append(self.current_angle)
        # self.ax.clear()
        # self.ax.plot(self.ys, self.xs)
        # plt.xticks(rotation=45, ha='right')
        # plt.subplots_adjust(bottom=0.30)
        # plt.draw()
        # plt.pause(0.1)
        # plt.show(block = False)

    
class scan_subscriber(Node):
    def __init__(self):
        super().__init__('scanner')
        #to address policy difference when subscribing to /scan topic in real life, may cause reliability issue down the road.
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)        
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback,qos_policy)
        
    def scan_callback(self,scan_data):
        print(scan_data.ranges[10])   
        

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


    #msg.angular_velocity.z > 0 turn left/counter-clockwise and vice-versa
    #msg.orientation.z returns a triangle waveform when plotted against angle the if/elif statement transforms it to a saw-tooth waveform
    #by inverting the negative slopes of the triangle
    ####WARNING#### from a high angular velocity to when it stops it may accidentally invert the wrong values due to IMU giving wrong velocity values
    #due to momentum

#returns the shortest turning angle in degrees from -180 to 180, 180 degree turn will always be anti-clockwise
def shortest_angle(current_angle,target_angle):
    return (target_angle - current_angle + 540)%360 - 180


def motion(node):

    #target_angle = int(input("Target angle (deg): "))
    twist = Twist()
    angular_vel = 0.2       #arbitrary value, max 2.84
    angular_error= 0.05     #arbitrary value,
    control_angular = 0
    linear_error = 3
    control_linear = 0
    linear_vel = 0.05
    angular_print = "To print rotational direction"
    

    for i in range(0,len(node.astar_path['angle'])):
        #orientation travel
        target_angle = node.astar_path['angle'][i]
        print(target_angle)
        time.sleep(2)
        try:
            while (np.abs(target_angle - node.orientation_z_deg) > angular_error):
                rclpy.spin_once(node)
                print(node.orientation_z_deg)
                #turn clockwise
                if shortest_angle(node.orientation_z_deg,target_angle) > 0:     # + value, clockwise
                    target_angular = angular_vel
                #turn anti-clockwise
                elif shortest_angle(node.orientation_z_deg,target_angle) < 0:   # - value, anti-clockwise
                    target_angular = -angular_vel

                control_angular = make_simple_profile(control_angular,target_angular,0.05)  #simple profile provided by TurtleBot3 creators, can replace it with own speed controller
                twist.angular.z = control_angular
                node.teleop_publisher.publish(twist)
        except:
            print("Failed, no idea why")
        finally:    #to stop the rotation
            twist.angular.z = 0.0 
            node.teleop_publisher.publish(twist)

        #linear travel
        try:
            while(np.sqrt(np.sum((node.location['turtlebot3_location'][-1] - node.astar_path['coordinate'][i])**2))>linear_error):  #need to convert coordinates to be same scale
                print(np.sqrt(np.sum((node.location['turtlebot3_location'][-1] - node.astar_path['coordinate'][i])**2)),node.location['turtlebot3_location'][-1])
                rclpy.spin_once(node)
                twist.linear.x = linear_vel
                node.teleop_publisher.publish(twist)

        finally:
            twist.linear.x = 0.0
            node.teleop_publisher.publish(twist)

    #old code, to be deleted if not used 1/07/2024    
    # for i in range(0,len(node.astar_path['angle'])):
    #     print(node.astar_path['angle'][i])
    #     #orientation
    #     try:
    #         while ((np.mod(node.astar_path['angle'][i]- node.orientation_z+3,2)-1)>angular_sensitivity or (np.mod(node.astar_path['angle'][i]- node.orientation_z+3,2)-1)<-angular_sensitivity ):
    #             rclpy.spin_once(node)

    #             if ((np.mod(node.astar_path['angle'][i]- node.orientation_z+3,2)-1)>angular_sensitivity):
    #                 twist.linear.x = 0.0
    #                 target_angular= angular_vel
    #                 if angular_print != "left":
    #                     angular_print = "left"
    #                     print(angular_print)
                        
    #             elif ((np.mod(node.astar_path['angle'][i]- node.orientation_z+3,2)-1)<-angular_sensitivity):
    #                 twist.linear.x = 0.0
    #                 target_angular= -angular_vel
    #                 if angular_print != "right":
    #                     angular_print = "right"
    #                     print(angular_print)
    #             else:
    #                 twist.linear.x=0.0
    #                 twist.angular.z=0.0
    #                 #self.current_angle = np.mod(self.current_angle + self.target_angle,360)
                    
    #             control_angular = make_simple_profile(control_angular,target_angular,0.05)
    #             twist.angular.z = control_angular
    #             node.teleop_publisher.publish(twist)
    #     except:
    #         print("Failed, no idea why")
    #     finally:
    #         twist.linear.x=0.0
    #         twist.angular.z=0.0
    #         node.teleop_publisher.publish(twist)

    #     #linear travel
    #     try:
    #         start_pose = node.pose_data
    #         while (time.time()-start_time<node.astar_path['coordinate'][i]):  #20/6/2024 will work to get the coordinate directly from the top camera to create a feedback
                
    #             control_linear = make_simple_profile(control_linear,target_linear,(LIN_VEL_STEP_SIZE / 2.0))
    #             twist.linear.x = control_linear
    #             node.teleop_publisher.publish(twist)
                
    #     finally:
    #         twist.linear.x=0.0
    #         twist.angular.z=0.0
    #         node.teleop_publisher.publish(twist)
    #old code 1/07/2024

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    #scan = scan_subscriber()
    node = teleop()
    scan = scan_subscriber()    
    #cli = MinimalClientAsync()     #14/6/2024 creating a client service connection between astarservice node and teleop node
    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    motion_call_times = 0
    motion_called = 0
    try:
        print(msg)
        while(1):

            

            # if path == 0:
            #     path = cli.send_request("request A*")
            #     print(path)
            # rclpy.spin_once(node)
                           
            #rclpy.spin(scan)    
            rclpy.spin_once(node) 
            motion(node)
            #if (type(node.astar_path[0]) != str) & (motion_called < motion_call_times):
            if (node.astar_path['angle'] != []) & (motion_called < motion_call_times):  
                motion(node) 
                motion_called =+ 1
            #key = get_key(settings)
            key = 0
            
            if key == 'w':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            node.teleop_publisher.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        node.teleop_publisher.publish(twist)
        
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
