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

"""
30/7/26 Working, used in static navigation test. Records,
'Trip Number', 'Start Time', 'Elapsed Time (s)','x','y','yaw','target_x','target_y','status','wait_count'
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
        self.create_subscription(Path,'plan',self.plan_poses_callback,10)
        self.create_subscription(PoseStamped,'pose',self.pose_callback, 1)
        # self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.pose_callback,1)

        self.trip = 1
        self.paths:list[Path] = []
        self.times = []  # List to store times for each trip
        self.durations = []
        self.final_poses = []
        self.final_poses_tf = []
        self.pose = PoseStamped()
        self.status = []
        self.wait_count = []
        self.wait = 0
        self.time = self.get_clock().now()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_goal_handle = None
        self.waiting_for_result = True

    def wait_status_callback(self, data:GoalStatusArray):
        self.wait +=1
        # self.get_logger().info(f'count {self.wait}')

    def pose_callback(self, data:PoseStamped):
        self.pose = data
        # self.get_logger().info(f'Current pose: x={data.pose.position.x}, y={data.pose.position.y}')

    # def pose_callback(self, data:PoseWithCovarianceStamped):
    #     self.pose = data.pose

    def plan_poses_callback(self, data:Path):
        data.header.frame_id = f'{self.trip}'
        self.paths.append(data)
        # self.get_logger().info(f'{self.paths[0].header.frame_id}')

        
    def send_goal(self, goal_pose: PoseStamped):

        self.waiting_for_result = True 
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Start timing
        start_time = time.time()
        self.times.append(self.get_clock().now())
        
        # Send the goal and set up callbacks
        self.get_logger().info('Sending goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return start_time
    
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
        result = future.result().result
        status = future.result().status
        
        end_time = time.time()
        elapsed_time = end_time - self.start_time
        
        if status == 4:  # 4 corresponds to SUCCEEDED
            t_now = self.get_clock().now()
            self.get_logger().info(f'{t_now.seconds_nanoseconds()} Movement succeeded! Took: {elapsed_time:.2f} seconds')
           
            
        else:
            self.get_logger().error(f'Movement failed! Status: {status}')

        self.waiting_for_result = False
        # self.plan_nodes.append(self.nodes)
        # time.sleep(0.5)
        # self.pose_callback()
        # pose = self.get_current_pose_from_tf()
        # self.get_logger().info(f'{self.get_clock().now()}, {self.time}')
        


        # time.sleep(3)  # Wait for 3 seconds to ensure tf is updated

        # position = self.get_current_pose_from_tf().pose.position
        # self.get_logger().info(f'TF pose: x={position.x:.4f}, y={position.y:.4f}')
        # self.final_poses_tf.append(position)
        self.final_poses.append(self.pose)
        self.get_logger().info(f'Final pose: x={self.pose.pose.position.x:.4f}, y={self.pose.pose.position.y:.4f}')
        self.durations.append(elapsed_time)
        self.status.append(status)
        self.wait_count.append(self.wait)
        self.trip += 1
        self.wait = 0        
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # You can process feedback here if needed
        # For example: self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')
    
    def log_times_to_csv(self, target_poses, filename='movement_times.csv'):
        # Write the logged times to a CSV file
        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Trip Number', 'Start Time', 'Elapsed Time (s)','x','y','yaw','target_x','target_y','status','wait_count'])
            for i, ( start_time, duration, final_pose,status,wait_count) in enumerate(zip(self.times, self.durations, self.final_poses,self.status,self.wait_count), start=1):
                x = final_pose.pose.position.x
                y = final_pose.pose.position.y
                # tf_x = tf_pose.x
                # tf_y = tf_pose.y
                eular_xyz = euler_from_quaternion(final_pose.pose.orientation.x,final_pose.pose.orientation.y,final_pose.pose.orientation.z,final_pose.pose.orientation.w)
                
                writer.writerow([i, start_time.to_msg().sec, duration,x,y,eular_xyz[2],target_poses[i-1][0],target_poses[i-1][1],status,wait_count])
        self.get_logger().info(f'Times have been logged to {filename}')

    def log_path_to_csv(self, filename='path.csv'):
        # Write the logged times to a CSV file
        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Trip','Path','TimeStamp', 'x', 'y'])
            for i, path in enumerate(self.paths, start=1):
                time = path.header.stamp.sec + path.header.stamp.nanosec * 1e-9

                for pose in path.poses:
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    trip = path.header.frame_id

                    writer.writerow([trip,i,time, x, y])
        self.get_logger().info(f'Times have been logged to {filename}')

    #You can get robot pose from transform, though tested it aint as accurate as from pose topic
    #Redundant code.
    # def get_current_pose_from_tf(self):
    #     """Get current robot pose using tf2"""
    #     try:
    #         # Get transform from map to base_link (or robot frame)
    #         transform = self.tf_buffer.lookup_transform(
    #             'map',  # target frame
    #             'base_footprint',  # source frame (adjust to your robot's base frame)
    #             rclpy.time.Time(),  # get latest
    #             timeout=rclpy.duration.Duration(seconds=1.0)
    #         )
            
    #         # Convert transform to PoseStamped
    #         pose_stamped = PoseStamped()
    #         pose_stamped.header.frame_id = 'map'
    #         pose_stamped.header.stamp = transform.header.stamp
    #         pose_stamped.pose.position.x = transform.transform.translation.x
    #         pose_stamped.pose.position.y = transform.transform.translation.y
    #         pose_stamped.pose.position.z = transform.transform.translation.z
    #         pose_stamped.pose.orientation = transform.transform.rotation
            
    #         return pose_stamped
            
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         self.get_logger().error(f'Could not get transform: {e}')
    #         return None


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
    
    # Define the start and goal points (PoseStamped)
    point_a = PoseStamped()
    
    # Set the coordinates for point A and B
    point_a.header.frame_id = 'map'  # Set frame_id
    point_a.pose.orientation.w = 1.0  # No rotation
    # Different maps have different point of origin, offsets are used to synchronize origins across maps.
    # I think you can offset the origin in the map.yaml file that is generated
    offset = [0,0,0]
    # offset = [1.247019,	5.555149, -0.0350734] #sim offset slam
    # offset = [1.66238, -3.89615, -0.0350734] #real offset slam
    # offset = [1.70533, -3.76842, -0.0350734]

    # Number of repetitions for the round trip
    num_repeats = 40
    dynamic_poses_bottom = [[3.0,1.0],[1.5,0.5]]
    dynamic_poses_top = [[3.0,7.0],[1.0,5.0],[3.0,3.0],[1.0,7.0]]
    random_index = 0
    target_poses = []

    try:
        at_bottom_group = False
        for i in range(num_repeats):
                
            if at_bottom_group:
                random_index = np.random.randint(0,len(dynamic_poses_top))
                point_a.pose.position.x = dynamic_poses_top[random_index][0] + offset[0]
                point_a.pose.position.y = dynamic_poses_top[random_index][1] + offset[1]
                target_poses.append([point_a.pose.position.x , point_a.pose.position.y])
            else:
                random_index = np.random.randint(0,len(dynamic_poses_bottom))
                point_a.pose.position.x = dynamic_poses_bottom[random_index][0] + offset[0]
                point_a.pose.position.y = dynamic_poses_bottom[random_index][1] + offset[1]
                target_poses.append([point_a.pose.position.x , point_a.pose.position.y])
            # A to B
            navigation_timer.get_logger().info(f'Starting trip {i+1} to x: {point_a.pose.position.x}, y: {point_a.pose.position.y}')
            navigation_timer.start_time = navigation_timer.send_goal(point_a)
            
            # Wait until the action is complete
            while navigation_timer.waiting_for_result:
                rclpy.spin_once(navigation_timer)
                time.sleep(0.1)  # Small sleep to avoid CPU overuse
            
            at_bottom_group = not at_bottom_group

            input("Press Enter to continue...")
        
        # After the test, log the times to CSV

        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        navigation_timer.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        
        navigation_timer.get_logger().info('Saving data...')
        navigation_timer.log_times_to_csv(target_poses)
        navigation_timer.log_path_to_csv()
        # Shutdown ROS after execution
        navigation_timer.get_logger().info('Shutting down...')
        # navigation_timer.log_times_to_csv()  # Save what we have
        
        # rclpy.shutdown()

if __name__ == '__main__':
    main()