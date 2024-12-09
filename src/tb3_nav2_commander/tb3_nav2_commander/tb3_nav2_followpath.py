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
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State


"""
Basic navigation demo to follow a given path after smoothing
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
    initial_pose.pose.position.y = 0.59 
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer='localizer')

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 3.0
    goal_pose.pose.orientation.w = 1.0

    # Get the path, smooth it
    path = navigator.getPath(initial_pose, goal_pose)
    print("Path found: ", path)
    smoothed_path = navigator.smoothPath(path)
    print("Smooth path found: ", smoothed_path)
    # Follow path
    navigator.followPath(smoothed_path)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated distance remaining to goal position: '
                + '{0:.3f}'.format(feedback.distance_to_goal)
                + '\nCurrent speed of the robot: '
                + '{0:.3f}'.format(feedback.speed)
            )

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

    navigator.cancelTask()
    navigator.cancelTask()
    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()