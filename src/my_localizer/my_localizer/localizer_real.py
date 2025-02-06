import rclpy
import rclpy.qos
import cv2 as cv
import numpy as np
import math

from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16MultiArray

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from ultralytics import YOLO
from my_custom_msgs.msg import Bbox


class localizer(Node):

    def __init__(self):
        super().__init__('localizer')
        self.br = CvBridge()
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                    durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth = 10
                                    )
        
        cv.namedWindow('image',cv.WINDOW_NORMAL)

        self.model = YOLO("/home/tarumt2204/YOLOv8_ws/runs/detect/TB3_train_v3/weights/best.pt")
        self.current_frame = 0
        self.pose_xy_pixels = [0,0]
        self.pose_xy_homo = Pose()
        self.pose_xy_pixels_homo = Int16MultiArray()
        self.bounding_box = Bbox()

        self.homo_resolution = 3.54/1203
        self.declare_parameter('show_homographic_region',False)
        self.show_homographic_region = self.get_parameter('show_homographic_region').get_parameter_value().bool_value
        self.pose_tf_publisher = TransformBroadcaster(self,10)
        self.camera_subscription = self.create_subscription(Image,'camera/image_raw', self.camera_callback,10)
        self.imu_subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.pose_publisher = self.create_publisher(Pose,'pose',qos_policy)
        self.pose_pixel_publisher = self.create_publisher(Int16MultiArray,'pose_pixel',qos_policy)
        self.bounding_box_publisher = self.create_publisher(Bbox,'bounding_box',10)
        self.pose_timer = self.create_timer(0.033,self.pose_timer_callback)
        

    def imu_callback(self,data:Imu):
        try:

            self.pose_xy_homo.orientation.x = data.orientation.x
            self.pose_xy_homo.orientation.y = data.orientation.y
            self.pose_xy_homo.orientation.z = data.orientation.z
            self.pose_xy_homo.orientation.w = data.orientation.w
            print("Angle: ", euler_from_quaternion(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))

        except Exception as e:
            print(e)

        finally:
            return
    
    def camera_callback(self,data:Image):
        
        try:

            self.current_frame = self.br.imgmsg_to_cv2(data)
            self.localization_yolov8()  #this function is tightly integrated with the class instance, lots of changes needed to be standalone
            homographic_img = self.homographic(self.pose_xy_pixels)
            




        except Exception as e:
            print(e)

        finally:
            return
        
    def pose_timer_callback(self):
        # self.get_logger().info(f"Node '{self.get_name()}' is in state '")
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

        odom_to_basefootprint = TransformStamped()
        odom_to_basefootprint.header.stamp = self.get_clock().now().to_msg()
        odom_to_basefootprint.header.frame_id = 'odom'
        odom_to_basefootprint.child_frame_id = 'base_footprint'
        odom_to_basefootprint.transform.translation.x = self.pose_xy_homo.position.x
        odom_to_basefootprint.transform.translation.y = self.pose_xy_homo.position.y 
        odom_to_basefootprint.transform.translation.z = 0.0
        

        odom_to_basefootprint.transform.rotation.x = self.pose_xy_homo.orientation.x
        odom_to_basefootprint.transform.rotation.y = self.pose_xy_homo.orientation.y
        odom_to_basefootprint.transform.rotation.z = self.pose_xy_homo.orientation.z
        odom_to_basefootprint.transform.rotation.w = self.pose_xy_homo.orientation.w

        
        self.pose_tf_publisher.sendTransform(odom_to_basefootprint)
        self.pose_tf_publisher.sendTransform(map_to_odom)
        #pose in meter with xy coordinates
        self.pose_publisher.publish(self.pose_xy_homo)
        #pose in pixels with xy coordinates (opencv uses row col or y x)
        self.pose_pixel_publisher.publish(self.pose_xy_pixels_homo)
        #1 box = 2 coordinates / 4 values order top left bottom right
        self.bounding_box_publisher.publish(self.bounding_box)

    def localization_yolov8(self):
        """
        Input: image
        Output: annotated image of detection, pose, bounding box coordinates
        Description: It doesnt return anything but saves them to the instance variables
        """
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
        self.bounding_box.data = np.array(tb3_box_TLBR,dtype=np.int16).tolist()
        self.bounding_box.header.stamp = self.get_clock().now().to_msg()
        
        #middle point of x/column
        x = (tb3_box_TLBR[0]+tb3_box_TLBR[2])/2
        y = tb3_box_TLBR[3]
        self.pose_xy_pixels = np.array([x,y],dtype=np.int16).tolist()
        # self.pose.position.x = self.pose_pixel.data[0]*3.54/1203 
        # self.pose.position.y = self.pose_pixel.data[0]
        #get the lower higher point of the y/row (because 0,0 is top left)
        # self.pose_xy_pixels[1] = tb3_box_TLBR[3]

    def homographic(self,pose):

        #specify the 4 corner coordinates are col row / x y. Row major order (Z shape)
        pts1 = np.float32([[599,98],[938,104],[72,532],[1275,637]])
        pts2 = np.float32([[0,0],[1203,0],[0,2869],[1203,2869]])
        M = cv.getPerspectiveTransform(pts1,pts2)
        # homo_transform = cv.warpPerspective(img,M,(1203,2869))
        pose = np.append(pose,1)



        pose_transformed = M.dot(np.int16(pose))
        x_pix = np.int16(pose_transformed[0]/pose_transformed[2])
        y_pix = np.int16(pose_transformed[1]/pose_transformed[2])

        self.pose_xy_pixels_homo.data = np.array([x_pix,y_pix]).tolist()

        self.pose_xy_homo.position.x = self.pose_xy_pixels_homo.data[0]*self.homo_resolution
        self.pose_xy_homo.position.y = (2869 - self.pose_xy_pixels_homo.data[1])*self.homo_resolution



        # homo_transform = cv.circle(homo_transform,self.pose_xy_pixels_homo.data,5,(255,0,0),-1,cv.LINE_4)

        if self.show_homographic_region:
            pts1 = np.array([pts1[0],pts1[2],pts1[3],pts1[1]],np.int32)
            pts1 = pts1.reshape((-1, 1, 2))
            
            self.annotated_frame = cv.polylines(self.annotated_frame,[pts1],1,[0,0,255],2,cv.LINE_4)
            cv.imshow('image',self.annotated_frame)
            cv.waitKey(1)

        return 
    
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


def main(args=None):
    rclpy.init(args=args)
    node = localizer()
    rclpy.spin(node)

    node.destroy_node()

if __name__== '__main__':
    main()