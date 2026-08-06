import rclpy
import time
import csv
import math
import numpy as np
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
import os
from rclpy.task import Future, Task
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_ros
import rclpy.time

"""
29/7/26 Dynamic navigation test moves turtleBot3 between specified points
Records, trip number, start time, elapsed time, final pose, target pose, status and wait count in csv file
You must manually uncomment pose_callback at line 38 & 73,74 for ExPeNav2 and line 39 & 75,76 for Nav2
Only one pose_callback should be uncommented at a time, depending on the system you are using
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
        # self.create_subscription(PoseStamped,'pose',self.pose_callback,10) #Uncomment this line to use with ExPeNav2 system
        self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.pose_callback,1)    #Uncomment this line to use with Nav2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.trip = 1
        self.paths:list[Path] = []
        self.times = []  # List to store times for each trip
        self.durations = []
        self.final_poses = []
        self.pose = PoseStamped().pose
        self.status = []
        self.wait_count = []
        self.wait = 0
        
        self.dir = os.path.join(os.getcwd(),"nav_dynamic_test")
        os.makedirs(self.dir, exist_ok=True)

        self.current_goal_handle = None
        self.waiting_for_result = True
        self.reset_trip_data()


    def reset_trip_data(self):
        self.times = []
        self.durations = []
        self.final_poses = []
        self.status = []
        self.wait_count = []
        self.wait = 0

    def wait_status_callback(self, data:GoalStatusArray):
        self.wait +=1

    #Uncomment this function to use with ExPeNav2 system
    # def pose_callback(self, data:PoseStamped):
    #     self.pose = data.pose

    #Uncomment this function to use with Nav2
    def pose_callback(self, data:PoseWithCovarianceStamped):
        self.pose = data.pose.pose

        
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
            self.get_logger().info(f'Movement succeeded! Took: {elapsed_time:.2f} seconds')
           
            
        else:
            self.get_logger().error(f'Movement failed! Status: {status}')

        self.waiting_for_result = False
        # self.plan_nodes.append(self.nodes)
        self.final_poses.append(self.pose)
        self.durations.append(elapsed_time)
        self.status.append(status)
        self.wait_count.append(self.wait)
        self.trip += 1
        self.wait = 0        
    
    def feedback_callback(self, feedback_msg:NavigateToPose.Feedback):
        feedback = feedback_msg.feedback
        # You can process feedback here if needed
        # For example: self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')
    
    def log_times_to_csv(self, target_poses, filename='movement_times.csv'):
        # Write the logged times to a CSV file
        with open(os.path.join(self.dir, filename), mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Trip Number', 'Start Time', 'Elapsed Time (s)','x','y','yaw','target_x','target_y','status','wait_count'])
            for i, ( start_time, duration, final_pose,status,wait_count) in enumerate(zip(self.times, self.durations, self.final_poses,self.status,self.wait_count), start=1):
                x = final_pose.position.x
                y = final_pose.position.y
                eular_xyz = euler_from_quaternion(final_pose.orientation.x,final_pose.orientation.y,final_pose.orientation.z,final_pose.orientation.w)
                
                writer.writerow([i, start_time.to_msg().sec, duration,x,y,eular_xyz[2],target_poses[i-1][0],target_poses[i-1][1],status,wait_count])
        self.get_logger().info(f'Times have been logged to {filename}')

    def log_trip_to_csv(self, target_pose, filename='movement_times.csv'):
        # Log the current trip's data to CSV
        with open(os.path.join(self.dir, filename), mode='a', newline='') as file:
            writer = csv.writer(file)
            # Write header only if file is empty
            if file.tell() == 0:
                writer.writerow(['Trip Number', 'Start Time', 'Elapsed Time (s)','x','y','yaw','target_x','target_y','status','wait_count'])
            for i, (start_time, duration, final_pose, status, wait_count) in enumerate(zip(self.times, self.durations, self.final_poses, self.status, self.wait_count), start=1):
                x = final_pose.position.x
                y = final_pose.position.y
                eular_xyz = euler_from_quaternion(final_pose.orientation.x, final_pose.orientation.y, final_pose.orientation.z, final_pose.orientation.w)
                writer.writerow([self.trip, start_time.to_msg().sec, duration, x, y, eular_xyz[2], target_pose[0], target_pose[1], status, wait_count])
        self.get_logger().info(f'Trip {self.trip} logged to {filename}')
        self.reset_trip_data()


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

def log_to_csv(filename: str, data: list[list], header: list = None):
    """
    Logs a 2D list to a CSV file.

    Args:
        filename (str): The path to the CSV file.
        data (list[list]): The 2D list to write (list of rows).
        header (list, optional): List of column names. If provided, written as the first row.
    """
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        if header:
            writer.writerow(header)
        writer.writerows(data)




def main(args=None):
    rclpy.init(args=args)
    navigation_timer = NavigationTimer()
    
    cmd_vel = Twist()
    speed = 0.22
    point_a = PoseStamped()
    
    # Set the coordinates for point A and B
    point_a.header.frame_id = 'map'  # Set frame_id
    point_a.pose.orientation.w = 1.0  # No rotation
    # Set the coordinates for point A and B
    start_poses = [[2.0, 6.0], [2.0, 5.5], [2.0, 5.0], [2.0, 4.5]]
    end_poses = [[2.0, 2.0], [2.0, 1.5], [2.0, 1.0], [2.0, 0.5]]
    # target_poses = []


    try:

        for i in range(1,35):
            input("Press Enter to start trip...")
            cmd_vel.linear.x = speed
            if i%2==1:
                pose = end_poses[3]
            else:
                pose = start_poses[3]
            point_a.pose.position.x = pose[0]
            point_a.pose.position.y = pose[1]
            navigation_timer.get_logger().info(f'Starting trip {i+1} to x: {point_a.pose.position.x}, y: {point_a.pose.position.y}')
            navigation_timer.start_time = navigation_timer.send_goal(point_a)


            # Wait until the action is complete
            while navigation_timer.waiting_for_result:

                rclpy.spin_once(navigation_timer)
                time.sleep(0.1)  # Small sleep to avoid CPU overuse   

            navigation_timer.log_trip_to_csv(pose)         


        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        navigation_timer.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        
        navigation_timer.get_logger().info('Saving data...')
        navigation_timer.get_logger().info('Shutting down...')


if __name__ == '__main__':
    main()