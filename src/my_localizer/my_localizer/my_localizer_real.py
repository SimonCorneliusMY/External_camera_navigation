import rclpy
import rclpy.qos
import cv2 as cv
import numpy as np
import math
import time
import csv
import os


from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import OccupancyGrid
from nav_2d_msgs.msg import Path2D

from ultralytics import YOLO

from tf2_ros import TransformBroadcaster
"""
24/01/13 can localize and map but cant navigate with nav2, switching to split mapping and localization into 2 nodes
"""
class top_camera(LifecycleNode):

    def __init__(self):
        super().__init__('localizer')



        self.current_frame = 0
        self.annotated_frame = 0
        self.imu_data= None
        self.br = None
        self.localization_tf = None
        self.timer = None
        self.imu_subscription = None
        self.camera_subscription = None

        self.model = YOLO("/home/tarumt2204/YOLOv8_ws/runs/detect/TB3_train_v3/weights/best.pt") 
        
        self.size = 76
        self.maze_ros = []
        self.maze_image = np.array([[[0]]])
        self.height = 0
        self.width = 0
        # TurtleBot3 pose from camera and homographic transform
        self.pose_xy_pixels = [0,0]
        self.pose_xy_homo = [0.0,0.0]
        self.pose_xy_pixels_homo = [0,0]

        self.homo_resolution = 3.54/1203
        self.yolo_results = []
        self.show_homographic_region = True
        
        
        self.imu_data= Imu()
        self.br = CvBridge()
        self.current_frame = 0

        np.set_printoptions(suppress=True,precision=2)  #suppress scientific notation and set to 2 decimal places for numpy value.
        qos_policy_reliable = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                    durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth = 10
                                    )
        qos_policy_best_effort = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                    durability = rclpy.qos.DurabilityPolicy.VOLATILE,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth = 10
                                    )
        

        self.camera_subscription = self.create_subscription(Image,'camera/image', self.camera_callback,qos_policy_best_effort)
        self.localization_tf = TransformBroadcaster(self)
        self.imu_subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid,'map',qos_policy_reliable)
        self.camera_pose = self.create_publisher(PoseStamped,'pose',qos_policy_reliable)
        self.map_timer = self.create_timer(0.01,self.map_callback)

 #lifecycle is used to activate simple commander api, normal node failed 27/9/2024  
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'")
        return TransitionCallbackReturn.SUCCESS
        
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'cleanup'")

        self.timer.destroy
        self.imu_subscription.destroy
        self.camera_subscription.destroy

        return TransitionCallbackReturn.SUCCESS
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'activate'")
        return TransitionCallbackReturn.SUCCESS
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'deactivate'")
        return TransitionCallbackReturn.SUCCESS
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'shutdown'")
        return TransitionCallbackReturn.SUCCESS
 #lifecycle is used to activate simple commander api 27/9/2024    


    def map_callback(self):
        map = OccupancyGrid()

        map.header.frame_id = 'map'
        map.header.stamp = self.get_clock().now().to_msg()
        map.info.height = self.height
        map.info.width = self.width
        map.info.resolution = 3.54/1203    #4m = 1044 pixels
        #map's pose
        
        map.info.origin.orientation.x = 0.0
        map.info.origin.orientation.y = 0.0
        map.info.origin.orientation.z = 0.0
        map.info.origin.orientation.w = 1.0
        map.info.origin.position.x =  0.0 #manually obtained through trial and error, will fix later 7/10/2024
        map.info.origin.position.y = 0.0    #1080*4/1044
        map.info.origin.position.z = 0.0         
        map.data = self.maze_ros

        self.map_publisher.publish(map)


        odom_to_basefootprint = TransformStamped()
        odom_to_basefootprint.header.stamp = self.get_clock().now().to_msg()
        odom_to_basefootprint.header.frame_id = 'odom'
        odom_to_basefootprint.child_frame_id = 'base_footprint'
        odom_to_basefootprint.transform.translation.x = self.pose_xy_homo[0]
        odom_to_basefootprint.transform.translation.y = self.pose_xy_homo[1] #+1080*4/1044   #1080*4/1044 added because global costmap not same as map 24/10/2024
        odom_to_basefootprint.transform.translation.z = 0.0

        

        odom_to_basefootprint.transform.rotation.x = self.imu_data.orientation.x
        odom_to_basefootprint.transform.rotation.y = self.imu_data.orientation.y
        odom_to_basefootprint.transform.rotation.z = self.imu_data.orientation.z
        odom_to_basefootprint.transform.rotation.w = self.imu_data.orientation.w
        

        map_to_odom = TransformStamped()
        
 
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        

        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0
        self.localization_tf.sendTransform(map_to_odom)

        
        self.localization_tf.sendTransform(odom_to_basefootprint)


        #self.get_logger().info('Current pose: {}  orientation: "{}"'.format(pose[::-1],xyz) )

    def camera_callback(self, data):
        #convert ros2 image data type to opencv image array
        self.current_frame = self.br.imgmsg_to_cv2(data)
        self.localization_yolov8()
        self.mapping()

    def imu_callback(self,data:Imu):
        self.imu_data= data
        print("Angle: ", euler_from_quaternion(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))
        

    def localization_yolov8(self):

        if isinstance(self.current_frame, int):
            # Handle the case where current_frame is not a valid image
            return
        
        self.yolo_results = self.model.track(self.current_frame,conf = 0.7,verbose = False)  # Assuming this returns detections

        # Check if any objects are detected
        if not self.yolo_results or len(self.yolo_results[0].boxes) == 0:
            # No objects detected, return default values
            return

        #annotate image with bounding box no labels
        self.annotated_frame = self.yolo_results[0].plot(labels=False)
        #get the coordinates of the bounding box [top_left:row col , bottom_right: row col]
        tb3_box_TLBR = self.yolo_results[0].boxes.xyxy.reshape(-1)
        # tb3_pose = np.empty((2,),np.int16)
        #middle point of x/column
        self.pose_xy_pixels[0] = (tb3_box_TLBR[0]+tb3_box_TLBR[2])/2
        #get the lower higher point of the y/row (because 0,0 is top left)
        self.pose_xy_pixels[1] = tb3_box_TLBR[3]    
    


    def mapping(self)->tuple[list[np.int8],int,int]:
        #threshold values to isolate floor TODO: pass in as parameter

        # Check if the image is None or invalid (empty)
        if isinstance(self.current_frame, int):
            # Return default values (empty list and zeros for width and height)
            return

        homographic_call = True


        #need a quick way to create the boundary based on given RGB values
        lower = np.array([10,0,0])
        upper = np.array([30,255,255])
        # upper_mid_gray = np.array([180,0,155])
        # upper_gray = np.array([170,0,155])

        #isolating
        hsv = cv.cvtColor(self.current_frame,cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv,lower,upper)
        mask = cv.bitwise_not(mask)


        for bbox in self.yolo_results[0].boxes.xyxy:  # Iterate through detected objects in the first batch
            x1, y1, x2, y2 = bbox  # Coordinates and class info
            # Black out the object by setting the region inside the bounding box to black
            mask[int(y1):int(y2), int(x1):int(x2)] = 0

        mask[mask==255] = 100
      
        ## Tried smoothing the image, but loss too much information
        # kernel = np.ones((5, 5), np.uint8)
        # # Apply the Opening operation (erode -> dilate)
        # mask_open = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)


        if homographic_call == True:
            self.maze_image = self.homographic(mask,self.pose_xy_pixels)


        #flip horizontally opencv data is origin is top left while map data is bottom left
        maze_bw_flip = cv.flip(self.maze_image,0)
        #row major order, and make sure values are int8
        self.maze_ros = np.array(maze_bw_flip,np.int8).reshape(-1).tolist()
        self.width = self.maze_image.shape[1]
        self.height = self.maze_image.shape[0]



    def homographic(self, img,pose):

        #specify the 4 corner coordinates are col row / x y. Row major order (Z shape)
        pts1 = np.float32([[599,98],[938,104],[72,532],[1275,637]])
        pts2 = np.float32([[0,0],[1203,0],[0,2869],[1203,2869]])
        M = cv.getPerspectiveTransform(pts1,pts2)
        homo_transform = cv.warpPerspective(img,M,(1203,2869))
        pose = np.append(pose,1)



        pose_transformed = M.dot(np.int16(pose))

        self.pose_xy_pixels_homo[0] = np.int16(pose_transformed[0]/pose_transformed[2])
        self.pose_xy_pixels_homo[1] = np.int16(pose_transformed[1]/pose_transformed[2])
        self.pose_xy_homo[0] = self.pose_xy_pixels_homo[0]*self.homo_resolution
        self.pose_xy_homo[1] = (homo_transform.shape[0] - self.pose_xy_pixels_homo[1])*self.homo_resolution



        homo_transform = cv.circle(homo_transform,self.pose_xy_pixels_homo,5,(255,0,0),-1,cv.LINE_4)

        if self.show_homographic_region:
            pts1 = np.array([pts1[0],pts1[2],pts1[3],pts1[1]],np.int32)
            pts1 = pts1.reshape((-1, 1, 2))
            
            self.annotated_frame = cv.polylines(self.annotated_frame,[pts1],1,[0,0,255],5,cv.LINE_4)

        return homo_transform

def object_counter(bw_image):

    (contours, hierarchy) = cv.findContours(
        bw_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    return len(contours)

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
        xyz[2] = round(yaw_z*180/math.pi,2)
     
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

def create_file(filename:str,max_copies:int = 5) -> None:
    filename_iter = filename
    i=0
    while ((os.path.exists(filename) & os.path.exists(filename_iter)) & (i<max_copies)):
        i +=1
        x = str.partition(filename,'.')
        filename_iter = x[0] + '_' + str(i) + x[1] + x[2]
    if filename_iter != filename:
        filename = filename_iter
    open(filename,'x')

def csv_writer(filename,fields,data):   #24/5/2024 not tested
    with open(filename, 'w') as csvfile:
        # creating a csv writer object
        csvwriter = csv.writer(csvfile)
    
        # writing the fields
        csvwriter.writerow(fields)
    
        # writing the data rows
        csvwriter.writerows(data)


def main(args=None):
    rclpy.init(args=args)
    camera = top_camera()
    cv.namedWindow('camera',cv.WINDOW_NORMAL)   #cv.window_normal to allow readjusting of window size

    while 1:

        rclpy.spin_once(camera)

        
        cv.resizeWindow('camera',800,800)
        
        cv.imshow('camera',camera.annotated_frame)

        cv.waitKey(1)   #0 means it will continue to show the image till a key is pressed hence it wont loop the code, 1 means it waits 1ms then continue the loop

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    camera.destroy_node()

if __name__ == '__main__':
    main()