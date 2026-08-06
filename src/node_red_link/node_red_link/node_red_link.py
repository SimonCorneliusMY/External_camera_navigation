import rclpy
import time
import csv
import math
import numpy as np
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from rclpy.task import Future, Task
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.time
from std_msgs.msg import Float32MultiArray, String
from tf_transformations import quaternion_from_euler
from std_srvs.srv import Trigger

"""
30/7/26 Node to interface with node red client with ExPeNav2
Takes goal commands from node red and sends them to nav2 action server
Returns a string feedback to node red.
"""


class NavigationTimer(Node):
    def __init__(self):
        super().__init__('navigation_timer')
        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        self.create_subscription(GoalStatusArray, 'wait/_action/status',self.wait_status_callback,10)
        self.create_subscription(Float32MultiArray, "node_red", self.node_red_callback, 10)
        self.create_subscription(PoseStamped, 'target_pose_0', self.target_pose_callback, 10)
        self.create_subscription(String, 'latency_test', self.latency_callback, 10)
        self.latency_pub = self.create_publisher(String, 'node_red_feedback', 10)
        self.move_service = self.create_service(Trigger, 'move_to_target_pose', self.handle_move_to_target_pose)
        self.node_red_feedback_pub = self.create_publisher(String,"node_red_feedback",10)

        # self.create_subscription(PoseStamped,'pose',self.pose_callback, 1)


        self.wait = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.times = []

        self.current_goal_handle = None
        self.latest_target_pose = None
        self.waiting_for_result = True
        self.printed = False
        self.move_requested = False

    def latency_callback(self, msg:String):
        self.latency_pub.publish(String(data=f"{msg.data}"))

    def handle_move_to_target_pose(self, request, response):
        if self.latest_target_pose is not None:
            self.move_requested = True
            response.success = True
            response.message = "Sent robot to target pose."
        else:
            response.success = False
            response.message = "No target pose available."
        return response

    def target_pose_callback(self, msg: PoseStamped):
        self.latest_target_pose = msg
        if self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp) < rclpy.duration.Duration(seconds=5.0) and not self.printed:
            self.node_red_feedback_pub.publish(String(data=f"Target detected x: {msg.pose.position.x:.2f} y: {msg.pose.position.y:.2f}"))
            self.printed = True
            return
        

    def node_red_callback(self, msg:Float32MultiArray):
        # self.get_logger().info(f"node_red_callback {msg.data}")
        x,y,z,yaw = msg.data
        q = quaternion_from_euler(0.0, 0.0, math.radians(yaw))
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = float(z)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        # self.get_logger().info(f"node_red_callback {goal_pose}")
        self.send_goal(goal_pose)

    def wait_status_callback(self, data:GoalStatusArray):
        self.wait +=1
        # self.get_logger().info(f'count {self.wait}')


        
    def send_goal(self, goal_pose: PoseStamped):

        self.waiting_for_result = True        
        
        goal = NavigateToPose.Goal()
        
        goal.pose = goal_pose
        
        # Wait for action server
        msg = 'Waiting for action server 10 seconds...'
        self.node_red_feedback_pub.publish(String(data=msg)) 
        self.get_logger().info(msg)
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            msg = 'Action server is not available.'
            self.node_red_feedback_pub.publish(String(data=msg))           
            self.get_logger().error(msg)
            return None

        msg = 'Action server is available.'
        self.get_logger().info(msg)
            
        self.node_red_feedback_pub.publish(String(data=msg))


        # Start timing
        self.start_time = time.time()
        self.times.append(self.get_clock().now())
        
        # Send the goal and set up callbacks
        self.get_logger().info(f'Sending goal x: {goal_pose.pose.position.x:.2f} y: {goal_pose.pose.position.y:.2f}...')
        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future: Future):
        goal_handle:ClientGoalHandle = future.result()
        
        if not goal_handle.accepted:
            msg = 'Goal was rejected!'
            self.get_logger().error(msg)
            return
        msg = 'Goal accepted!'
        self.get_logger().info(msg)
        self.node_red_feedback_pub.publish(String(data=msg))
        self.current_goal_handle = goal_handle
        
        # Request the result

        result_future: Future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future: Future):
        result = future.result().result
        status = future.result().status
        
        end_time = time.time()
        elapsed_time = end_time - self.start_time
        
        if status == 4:  # 4 corresponds to SUCCEEDED
            # t_now = self.get_clock().now()
            msg = f'Movement succeeded! Took: {elapsed_time:.2f} seconds'
            self.get_logger().info(msg)         
        else:
            msg = f'Movement failed! Status: {status}'
            self.get_logger().error(msg)

        self.node_red_feedback_pub.publish(String(data=msg))
   
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # You can process feedback here if needed
        # For example: self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')
    

    def get_current_pose_from_tf(self):
        """Get current robot pose using tf2"""
        try:
            # Get transform from map to base_link (or robot frame)
            transform = self.tf_buffer.lookup_transform(
                'map',  # target frame
                'base_footprint',  # source frame (adjust to your robot's base frame)
                rclpy.time.Time(),  # get latest
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Convert transform to PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = transform.header.stamp
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation
            
            return pose_stamped
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return None


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


def main(args=None):
    rclpy.init(args=args)
    navigation_timer = NavigationTimer()
    goal = PoseStamped()
    goal.header.frame_id = 'map'


    try:
        while(rclpy.ok()):
            rclpy.spin_once(navigation_timer)
            if navigation_timer.move_requested and navigation_timer.latest_target_pose is not None:
                goal.header.stamp = navigation_timer.get_clock().now().to_msg()
                goal.pose = navigation_timer.latest_target_pose.pose
                goal.pose.position.y -= 0.5  # add buffer
                navigation_timer.get_logger().info(f'Goal position: {goal.pose.position}')
                navigation_timer.send_goal(goal)
                navigation_timer.move_requested = False

        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        navigation_timer.get_logger().error(f'Error occurred: {str(e)}')
    finally:

        navigation_timer.get_logger().info('Shutting down...')


if __name__ == '__main__':
    main()