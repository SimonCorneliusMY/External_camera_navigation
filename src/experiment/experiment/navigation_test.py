import rclpy
import time
import csv
import math
import numpy as np
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
import rclpy.action
from rclpy.task import Future, Task
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

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
        self.create_subscription(PoseStamped,'pose',self.pose_callback,10)

        self.trip = 1
        self.paths:list[Path] = []
        self.times = []  # List to store times for each trip
        self.durations = []
        self.final_poses = []
        self.pose = PoseStamped()
        self.status = []
        self.wait_count = []
        self.wait = 0

        self.current_goal_handle = None
        self.waiting_for_result = True

    def wait_status_callback(self, data:GoalStatusArray):
        self.wait +=1
        # self.get_logger().info(f'count {self.wait}')

    def pose_callback(self, data:PoseStamped):

        self.pose = data

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
def exhaustive_routes(num_poses:int):
    

    return 

def main(args=None):
    rclpy.init(args=args)
    navigation_timer = NavigationTimer()
    
    # Define the start and goal points (PoseStamped)
    point_a = PoseStamped()
    # point_b = PoseStamped()
    
    # Set the coordinates for point A and B
    point_a.header.frame_id = 'map'  # Set frame_id
    # point_a.pose.position.x = 2.0  #sim 2.08  #2.080344553
    # point_a.pose.position.y = 0.4 #sim 3.75  #3.757467163
    point_a.pose.orientation.w = 1.0  # No rotation
    

    # Number of repetitions for the round trip
    num_repeats = 30
    poses = [[3.0,1.0],[3.0,7.5],[0.3,4.5],[3.3,3.0],[1.2,0.5],[0.3,7.0]]
    dynamic_poses_bottom = [[3.0,1.0],[1.2,0.5]]
    dynamic_poses_top = [[3.0,7.5],[0.3,4.5],[3.3,3.0],[0.3,7.0]]
    prev_index = 0
    random_index = 0
    target_poses = []
    # try:
    #     for i in range(num_repeats):
    #         while prev_index == random_index:
    #             random_index = np.random.randint(0,len(poses)-1)

    #         point_a.pose.position.x = poses[random_index][0]  #sim 2.08  #2.080344553
    #         point_a.pose.position.y = poses[random_index][1] #sim 3.75  #3.757467163

    #         # A to B
    #         navigation_timer.get_logger().info(f'Starting trip {i+1}A: A to B')
    #         navigation_timer.start_time = navigation_timer.send_goal(point_a)
            
    #         # Wait until the action is complete
    #         while navigation_timer.waiting_for_result:
    #             rclpy.spin_once(navigation_timer)
    #             time.sleep(0.1)  # Small sleep to avoid CPU overuse
            
    #         prev_index = random_index
    #         # Wait a bit between goals
    #         target_poses.append(poses[random_index])
    #         input("Press Enter to continue...")
    try:
        at_bottom_group = False
        for i in range(num_repeats):
                
            if at_bottom_group:
                random_index = np.random.randint(0,len(dynamic_poses_top))
                point_a.pose.position.x = dynamic_poses_top[random_index][0]
                point_a.pose.position.y = dynamic_poses_top[random_index][1]
                target_poses.append(dynamic_poses_top[random_index])
            else:
                random_index = np.random.randint(0,len(dynamic_poses_bottom))
                point_a.pose.position.x = dynamic_poses_bottom[random_index][0]
                point_a.pose.position.y = dynamic_poses_bottom[random_index][1]
                target_poses.append(dynamic_poses_bottom[random_index])
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