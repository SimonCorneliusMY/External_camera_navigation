import rclpy
import rclpy.qos
import cv2 as cv
import numpy as np
import math
import time
from datetime import datetime

from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
from geometry_msgs.msg import  TransformStamped
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PoseStamped
from ultralytics import YOLO
from my_custom_msgs.msg import Bbox
from tf2_geometry_msgs import do_transform_pose_stamped
from ament_index_python.packages import get_package_share_directory
import os


#Publishes the pose and bounding box of the robot regardless of whether it is detected.
#This node currently takes the most cpu usage at 5%, name is pt_main no idea why its called that
#Greatest time taken is 0.07 used by yolo detection.
#GPU used is intel which doesnt support cuda, hence optimization using GPU can be done for improvements.
# 30/7/26 it works,

class FPSCounter:
    def __init__(self):
        self.frame_count = 0
        self.start_time = time.time()



class localizer(Node):

    def __init__(self):
        super().__init__('localizer')

        self.declare_parameter('show_homographic_region',True)
        self.declare_parameter('name','0')
        self.declare_parameter('record',False)
        self.declare_parameter('publish_pose_tf', False)
        self.declare_parameter('resolution',1.0)
        self.declare_parameter('homographic_ori_points', [612,134,903,169,163,457,1079,619])
        self.declare_parameter('homographic_transformed_points', [0,0,1203,0,0,2869,1203,2869])
        # self.declare_parameter('yolo_model_path', "/home/tarumt2204/YOLOv8_ws/runs/detect/TB3_train_sim_v2/weights/best.pt")
        self.declare_parameter('yolo_model_path', os.path.join(get_package_share_directory('my_localizer'),'weights','2880_best.pt'))
        self.declare_parameter('YOLO_confidence_threshold', 0.7)



        self.br = CvBridge()
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                    durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth = 10
                                    )
        

        # use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.homo_resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.model = YOLO(self.get_parameter('yolo_model_path').get_parameter_value().string_value)
        self.publish_pose_tf = self.get_parameter('publish_pose_tf').get_parameter_value().bool_value
        if self.publish_pose_tf:
            self.pose_tf_publisher = TransformBroadcaster(self,10)
        #limit printing info to once
        self.object_print = False

        self.record = self.get_parameter('record').get_parameter_value().bool_value
        self.current_frame = 0
        self.pose_xy_pixels = [0,0]
        self.pose_xy_homo = PoseStamped()
        self.pose_xy_pixels_homo = Int16MultiArray()
        self.bounding_box = Bbox()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.label = ''
        self.conf = Float32()
        self.tb3_radius = 0.1
        self.pose_msgs = []
        self.conf_msgs = []
        self.bbox_msgs = []
        self.target_msgs = []
        self.image_stamp = self.get_clock().now().to_msg()

        

    
        #TODO in progress for multicamera, input camera addresses and perform localization on both
        # self.declare_parameter('camera_addresses',['0'])
        self.show_homographic_region = self.get_parameter('show_homographic_region').get_parameter_value().bool_value
        self.name = self.get_parameter('name').get_parameter_value().string_value

        if self.record:
            #Warning make sure you have enough space, 30 min record was 700MB
            #Frame resolution needs to be the same for recording to work.
            fourcc = cv.VideoWriter_fourcc(*'XVID')
            self.out = cv.VideoWriter('output_' + self.name + '_' + datetime.now().strftime("%Y-%m-%d_%H-%M-%S")+'.avi', fourcc, 10.0, (1280,  720))

        homo_ori = self.get_parameter('homographic_ori_points').get_parameter_value().integer_array_value
        homo_transformed = self.get_parameter('homographic_transformed_points').get_parameter_value().integer_array_value


        self.pts1 = np.float32([[homo_ori[0],homo_ori[1]],[homo_ori[2],homo_ori[3]],[homo_ori[4],homo_ori[5]],[homo_ori[6],homo_ori[7]]])
        pts2 = np.float32([[homo_transformed[0],homo_transformed[1]],[homo_transformed[2],homo_transformed[3]],
                           [homo_transformed[4],homo_transformed[5]],[homo_transformed[6],homo_transformed[7]]])
        self.M = cv.getPerspectiveTransform(self.pts1,pts2)

        
        self.camera_subscription = self.create_subscription(Image,'camera_'+ self.name +'/image_raw', self.camera_callback,10)
        self.imu_subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped,'pose_' + self.name,qos_policy)
        self.confidence_publisher = self. create_publisher(Float32,'conf_' + self.name,10)
        self.target_pose_publisher = self. create_publisher(PoseStamped,'target_pose_' + self.name, 10)
        # self.pose_pixel_publisher = self.create_publisher(Int16MultiArray,'pose_pixel_' + self.name,qos_policy)
        self.bounding_box_publisher = self.create_publisher(Bbox,'bounding_box_' + self.name,10)
        # self.pose_timer = self.create_timer(0.033,self.pose_timer_callback)


    # To publish messages processed   
    def publish(self):
        if self.publish_pose_tf:
            odom_to_basefootprint = TransformStamped()
            odom_to_basefootprint.header.stamp = self.get_clock().now().to_msg()
            odom_to_basefootprint.header.frame_id = 'odom'
            odom_to_basefootprint.child_frame_id = 'base_footprint'
            odom_to_basefootprint.transform.translation.x = self.pose_xy_homo.pose.position.x
            odom_to_basefootprint.transform.translation.y = self.pose_xy_homo.pose.position.y 
            odom_to_basefootprint.transform.translation.z = 0.0

            odom_to_basefootprint.transform.rotation.x = self.pose_xy_homo.pose.orientation.x
            odom_to_basefootprint.transform.rotation.y = self.pose_xy_homo.pose.orientation.y
            odom_to_basefootprint.transform.rotation.z = self.pose_xy_homo.pose.orientation.z
            odom_to_basefootprint.transform.rotation.w = self.pose_xy_homo.pose.orientation.w
            
            self.pose_tf_publisher.sendTransform(odom_to_basefootprint)

        for msg in self.pose_msgs:
            self.pose_publisher.publish(msg)

        for msg in self.conf_msgs:
            self.confidence_publisher.publish(msg)
        
        for msg in self.bbox_msgs:
            self.bounding_box_publisher.publish(msg)

        for msg in self.target_msgs:
            self.target_pose_publisher.publish(msg)

        self.pose_msgs.clear()
        self.conf_msgs.clear()
        self.bbox_msgs.clear()
        self.target_msgs.clear()

   
    # Storing imu data for later use
    def imu_callback(self,data:Imu):
        try:
            self.pose_xy_homo.pose.orientation = data.orientation

            # print("Angle: ", euler_from_quaternion(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))

        except Exception as e:
            
            self.get_logger().info('%s',e)

        finally:
            return

    # Used to call required functions for localiser
    def camera_callback(self,data:Image):
        
        try:
            
            self.current_frame = self.br.imgmsg_to_cv2(data)
            self.image_stamp = data.header.stamp    
            cv.cvtColor(self.current_frame,cv.COLOR_BGR2RGB,self.current_frame)        
            self.localization_yolov8()  #this function is tightly integrated with the class instance, lots of changes needed to be standalone       
            # self.homographic()
            self.publish()

            if self.show_homographic_region:
                self.show_mapping_region()
                # self.get_logger().info('Showing homographic region')

            if self.record:
                self.record_frame()

        except Exception as e:

            self.get_logger().info(f'Error: {e}')

        finally:
            return
       

    def localization_yolov8(self):
        """
        Input: image
        Output: annotated image of detection, pose, bounding box coordinates
        Description: It doesnt return anything but saves them to the instance variables
        """
        if isinstance(self.current_frame, int):
            # Handle the case where current_frame is not a valid image
            self.get_logger().info("Invalid image")
            return 
        conf = Float32()
        bbox = Bbox()
        #TODO optimize, line below takes 0.07 secs to process try gpu
        conf_threshold = self.get_parameter('YOLO_confidence_threshold').get_parameter_value().double_value
        self.yolo_results = self.model.track(self.current_frame,conf = conf_threshold,verbose = False)  # Assuming this returns detections
        bbox.header.stamp = self.get_clock().now().to_msg()

        # Check if any objects are detected and print the message once
        if len(self.yolo_results[0].boxes) == 0:
            # No objects detected, return default values
            # self.get_logger().info("No objects detected")
            bbox.header.frame_id = 'No objects detected'
            self.bbox_msgs.append(bbox)
            self.pose_msgs.append(self.homographic(self.pose_xy_pixels,'No objects detected'))
            self.object_print = True
            # self.conf.data = 0.0
            
            return
        
        # Bad implementation of object identification
        object_count = {"TurtleBot3": 0, "chair": 0, "fire hydrant": 0, "bottle": 0}
        for result in self.yolo_results:
            for box in result.boxes:
                xyxy = box.xyxy[0].int().tolist()
                if self.model.names[int(box.cls)] == "TurtleBot3" or self.model.names[int(box.cls)] == "Turtlebot3-sim":
                    # object_count['TurtleBot3'] += 1
                    self.pose_xy_pixels = [int((xyxy[0] + xyxy[2])/2),int(xyxy[3])]
                    conf.data = box.conf.item()
                    bbox.data = xyxy
                    bbox.header.frame_id = self.model.names[int(box.cls)]
                    self.pose_msgs.append(self.homographic(self.pose_xy_pixels,self.model.names[int(box.cls)],object_count['TurtleBot3']))
                    self.conf_msgs.append(conf)
                    self.bbox_msgs.append(bbox)
                elif self.model.names[int(box.cls)] == "bottle":
                    object_count['bottle'] += 1
                    self.target_msgs.append(self.homographic([(xyxy[0] + xyxy[2])/2,xyxy[3]], "bottle", object_count['bottle']))    
                elif self.model.names[int(box.cls)] == "fire hydrant":
                    object_count['fire hydrant'] += 1
                    self.target_msgs.append(self.homographic([(xyxy[0] + xyxy[2])/2,xyxy[3]], "fire hydrant", object_count['fire hydrant']))

    def homographic(self,pose_ori:list,label='No label given', count=0):

        if count != 0:
            label = f"{label}_{count}"

        pose = np.append(pose_ori,1)
        pose_transformed = self.M.dot(np.int16(pose))
        x_pix = np.int16(pose_transformed[0]/pose_transformed[2])
        y_pix = np.int16(pose_transformed[1]/pose_transformed[2])

        # self.pose_xy_pixels_homo.data = np.array([x_pix,y_pix]).tolist()

        pose = PoseStamped()
        pose.pose.position.x = x_pix*self.homo_resolution
        pose.pose.position.y = y_pix*self.homo_resolution
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        transform = self.tf_buffer.lookup_transform('map',f"map_{self.name}",rclpy.time.Time())
        # self.pose_xy_homo = do_transform_pose_stamped(pose,transform)
        # self.pose_xy_homo.pose.position.y += self.tb3_radius
        pose_return = do_transform_pose_stamped(pose,transform)
        pose_return.header.frame_id = label
        pose_return.header.stamp = self.image_stamp
        return pose_return

        
    def show_mapping_region(self):
        #annotate image with bounding box no labels
        self.annotated_frame = self.yolo_results[0].plot(labels=True, boxes=True)
        if self.annotated_frame is not None and self.pose_msgs is not None:
            pts1 = np.array([self.pts1[0],self.pts1[2],self.pts1[3],self.pts1[1]],np.int32)
            pts1 = pts1.reshape((-1, 1, 2))
            # homo_transform = cv.warpPerspective(self.current_frame,self.M,(1112,1168))
            
            cv.cvtColor(self.annotated_frame,cv.COLOR_RGB2BGR,self.annotated_frame)
            self.annotated_frame = cv.polylines(self.annotated_frame,[pts1],1,[0,0,255],2,cv.LINE_4)
            self.annotated_frame = cv.circle(self.annotated_frame,self.pose_xy_pixels,2,(0,0,255),-1,cv.LINE_4)
            # cv.namedWindow('image' + self.name,cv.WINDOW_NORMAL)
            cv.imshow('image' + self.name,self.annotated_frame)
            cv.waitKey(1)

    def record_frame(self):
        self.out.write(self.current_frame)

def object_counter(bw_image):

    (contours, hierarchy) = cv.findContours(
        bw_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    return len(contours)

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

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


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

    node.out.release()
    cv.destroyAllWindows()
    node.destroy_node()

if __name__== '__main__':
    main()
