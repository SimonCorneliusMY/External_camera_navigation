#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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


from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State
import time
import math
import numpy as np




"""
Basic navigation demo to go to pose.
"""
class MyLifeCycleManager(Node):
    def __init__(self):
        super().__init__('MyLifeCycleManager')

        self.client_get_state = self.create_client(GetState, 'localizer/get_state')
        self.client_set_state = self.create_client(ChangeState,'localizer/change_state')


    def get_state(self):
        
        while not self.client_get_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(self.client_get_state.srv_name+' service not available, waiting again...')
            

        req = GetState.Request()       
        self.future = self.client_get_state.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)   
        return(self.future.result().current_state.id)

    def set_state(self, transition_id:Transition) -> None:
        while not self.client_set_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(self.client_set_state.srv_name+' service not available, waiting again...')

        req = ChangeState.Request()
        req.transition.id = transition_id
        self.future = self.client_set_state.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        
    # def my_change_state(self):
    #     if self.get_state() == State.PRIMARY_STATE_UNCONFIGURED:
    #         self.set_state(Transition.TRANSITION_CONFIGURE)
    #     elif 


    #def setstate_callback(self,req,res):

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        xyz = np.empty((3,))
        xyz[0] = roll_x*180/math.pi
        xyz[1] = pitch_y*180/math.pi
        xyz[2] = yaw_z*180/math.pi
     
        return xyz

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    #x,y,z,w order
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def main():
    rclpy.init()

    x = MyLifeCycleManager()
    if x.get_state() == 1:
        x.set_state(Transition.TRANSITION_CONFIGURE)
        x.set_state(Transition.TRANSITION_ACTIVATE)
    

    navigator = BasicNavigator()
    
    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 4.1
    initial_pose.pose.position.y = 0.59 +1080*4/1044
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    #navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer='localizer')

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()


    # Go to our demos first goal pose
    q = quaternion_from_euler(0,0,math.pi/2)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 3.0 +1080*4/1044
    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]

    # sanity check a valid path exists
    path = navigator.getPath(initial_pose, goal_pose)

    #navigator.goToPose(goal_pose)
    navigator.followPath(path)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################
        

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #     goal_pose.pose.position.x = 0.0
            #     goal_pose.pose.position.y = 0.0
            #     navigator.goToPose(goal_pose)
    navigator.cancelTask
    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()