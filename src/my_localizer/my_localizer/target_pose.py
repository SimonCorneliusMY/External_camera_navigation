import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle
import time
import math

"""
30/7/26 Send goal to nav2 to move robot to target pose. Target pose is received from localizer node.
Manual change of object name is needed, currently fire hydrant_1.
"""


class TargetPose(Node):
    def __init__(self):
        super().__init__('target_pose_node')
        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',callback_group=self.callback_group)
        self.target_subscription = self.create_subscription(PoseStamped, 'target_pose_0', self.target_callback, 10)
        self.target_poses: dict[str, PoseStamped] = {}
        self.start_time = 0.0
        self.current_goal_pose = PoseStamped()
        self.object = 'fire hydrant_1'
        self.pose_buffer = 1 # meters


    def target_callback(self, msg:PoseStamped):
        
        self.target_poses[msg.header.frame_id] = msg
        # if self.current_goal_pose.header.frame_id == None:
        #     self.current_goal_pose = self.target_poses[self.object]
        goal_dist_change = math.sqrt(
        (self.current_goal_pose.pose.position.x - self.target_poses[self.object].pose.position.x)**2 + \
        (self.current_goal_pose.pose.position.y - self.target_poses[self.object].pose.position.y)**2 )

        if goal_dist_change > 1.0:
            self.current_goal_pose = self.target_poses[self.object]
            self.current_goal_pose.header.frame_id = 'map'
            self.current_goal_pose.pose.position.y -= self.pose_buffer
            self.get_logger().info(f'{self.current_goal_pose.pose.position}')
            self.send_goal(self.current_goal_pose)

        


    def send_goal(self, goal_pose: PoseStamped):
        goal = NavigateToPose.Goal()
        
        goal.pose = goal_pose
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.start_time = time.time()
        # Send the goal and set up callbacks
        self.get_logger().info('Sending goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle:ClientGoalHandle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return
            
        self.get_logger().info('Goal accepted!')
        self.current_goal_handle = goal_handle
        
        # Request the result

        result_future: Future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        status = future.result().status
        
        end_time = time.time()
        elapsed_time = end_time - self.start_time
        
        if status == 4:  # 4 corresponds to SUCCEEDED
            self.get_logger().info(f'Movement succeeded! Took: {elapsed_time:.2f} seconds')
           
            
        else:
            self.get_logger().error(f'Movement failed! Status: {status}')

    def feedback_callback(self, feedback_msg):
        pass

    def cancel_goal(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = TargetPose()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
