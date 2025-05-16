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
from geometry_msgs.msg import Pose, TransformStamped,Vector3Stamped
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32
from nav_msgs.msg import Path

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PoseStamped
from ultralytics import YOLO
from my_custom_msgs.msg import Bbox
from tf2_geometry_msgs import do_transform_pose_stamped



#Publishes the pose and bounding box of the robot regardless of whether it is detected.
#This node currently takes the most cpu usage at 5%, name is pt_main no idea why its called that
#Greatest time taken is 0.07 used by yolo detection.
#GPU used is intel which doesnt support cuda, hence optimization using GPU can be done for improvements.
class localizer(Node):

    def __init__(self):
        super().__init__('localizer')

        self.declare_parameter('show_homographic_region',False)
        self.declare_parameter('name','0')
        self.declare_parameter('record',False)
        self.declare_parameter('publish_pose_tf', False)
        self.declare_parameter('resolution',1.0)
        self.declare_parameter('homographic_ori_points', [612,134,903,169,163,457,1079,619])
        self.declare_parameter('homographic_transformed_points', [0,0,1203,0,0,2869,1203,2869])


        self.br = CvBridge()
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                    durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth = 10
                                    )
        

        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.homo_resolution = self.get_parameter('resolution').get_parameter_value().double_value
        if use_sim_time:
            self.model = YOLO("/home/tarumt2204/YOLOv8_ws/runs/detect/TB3_train_sim_v2/weights/best.pt")
            # self.homo_resolution = 3.54/1203
        elif not(use_sim_time):
            self.model = YOLO("/home/tarumt2204/YOLOv8_ws/runs/detect/TB3_train_v4/weights/best.pt")
            # self.homo_resolution = 3.54/1203

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
        if self.record:
            #Warning make sure you have enough space, 30 min record was 700MB
            #Frame resolution needs to be the same for recording to work.
            fourcc = cv.VideoWriter_fourcc(*'XVID')
            self.out = cv.VideoWriter('output_'+ datetime.now().strftime("%Y-%m-%d_%H-%M-%S")+'.avi', fourcc, 10.0, (1280,  720))
        

    
        #TODO in progress for multicamera, input camera addresses and perform localization on both
        # self.declare_parameter('camera_addresses',['0'])
        self.show_homographic_region = self.get_parameter('show_homographic_region').get_parameter_value().bool_value
        self.name = self.get_parameter('name').get_parameter_value().string_value

        homo_ori = self.get_parameter('homographic_ori_points').get_parameter_value().integer_array_value
        homo_transformed = self.get_parameter('homographic_transformed_points').get_parameter_value().integer_array_value


        self.pts1 = np.float32([[homo_ori[0],homo_ori[1]],[homo_ori[2],homo_ori[3]],[homo_ori[4],homo_ori[5]],[homo_ori[6],homo_ori[7]]])
        pts2 = np.float32([[homo_transformed[0],homo_transformed[1]],[homo_transformed[2],homo_transformed[3]],[homo_transformed[4],homo_transformed[5]],[homo_transformed[6],homo_transformed[7]]])
        self.M = cv.getPerspectiveTransform(self.pts1,pts2)

        
        self.camera_subscription = self.create_subscription(Image,'camera_'+ self.name +'/image_raw', self.camera_callback,10)
        self.imu_subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped,'pose_' + self.name,qos_policy)
        self.confidence_publisher = self. create_publisher(Float32,'conf_' + self.name,10)
        
        # self.pose_pixel_publisher = self.create_publisher(Int16MultiArray,'pose_pixel_' + self.name,qos_policy)
        self.bounding_box_publisher = self.create_publisher(Bbox,'bounding_box_' + self.name,10)
        # self.pose_timer = self.create_timer(0.033,self.pose_timer_callback)


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
        # self.pose_tf_publisher.sendTransform(map_to_odom)
        #pose in meter with xy coordinates

        self.pose_xy_homo.header.stamp = self.get_clock().now().to_msg()
        self.pose_xy_homo.header.frame_id = self.label
        self.pose_publisher.publish(self.pose_xy_homo)
        self.confidence_publisher.publish(self.conf)

        #pose in pixels with xy coordinates (opencv uses row col or y x)
        # self.pose_pixel_publisher.publish(self.pose_xy_pixels_homo)
        #1 box = 2 coordinates / 4 values order top left bottom right
        self.bounding_box.header.frame_id = self.label
        self.bounding_box_publisher.publish(self.bounding_box)            

    def imu_callback(self,data:Imu):
        try:
            self.pose_xy_homo.pose.orientation = data.orientation

            # print("Angle: ", euler_from_quaternion(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))

        except Exception as e:
            
            self.get_logger().info('%s',e)

        finally:
            return
    
    def camera_callback(self,data:Image):
        
        try:
            
            self.current_frame = self.br.imgmsg_to_cv2(data)           
            cv.cvtColor(self.current_frame,cv.COLOR_BGR2RGB,self.current_frame)        
            self.localization_yolov8()  #this function is tightly integrated with the class instance, lots of changes needed to be standalone       
            self.homographic(self.pose_xy_pixels)
            self.publish()

        except Exception as e:

            self.get_logger().info(f'Error: {e}')

        finally:
            return


    def pose_timer_callback(self):
        # self.get_logger().info(f"Node '{self.get_name()}' is in state '")
        # I feel like i should have done map to basefootprint and left the odom transform in the turtlebot3 on, 240219 Simon
        #Tried the above, didn't work, still require map to odom tf, shifted to static tf
        # map_to_odom = TransformStamped()
        # map_to_odom.header.stamp = self.get_clock().now().to_msg()
        # map_to_odom.header.frame_id = 'map'
        # map_to_odom.child_frame_id = 'odom'
        # map_to_odom.transform.translation.x = 0.0
        # map_to_odom.transform.translation.y = 0.0
        # map_to_odom.transform.translation.z = 0.0
        # map_to_odom.transform.rotation.x = 0.0
        # map_to_odom.transform.rotation.y = 0.0
        # map_to_odom.transform.rotation.z = 0.0
        # map_to_odom.transform.rotation.w = 1.0

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
        # self.pose_tf_publisher.sendTransform(map_to_odom)
        #pose in meter with xy coordinates

        self.pose_xy_homo.header.stamp = self.get_clock().now().to_msg()
        self.pose_xy_homo.header.frame_id = self.label
        self.pose_publisher.publish(self.pose_xy_homo)

        #pose in pixels with xy coordinates (opencv uses row col or y x)
        # self.pose_pixel_publisher.publish(self.pose_xy_pixels_homo)
        #1 box = 2 coordinates / 4 values order top left bottom right
        self.bounding_box.header.frame_id = self.label
        self.bounding_box_publisher.publish(self.bounding_box)

        


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

        #TODO optimize, line below takes 0.07 secs to process try gpu
        self.yolo_results = self.model.track(self.current_frame,conf = 0.2,verbose = False)  # Assuming this returns detections
        self.bounding_box.header.stamp = self.get_clock().now().to_msg()

        # self.get_logger().info(f"Confidence: {self.yolo_results[0].boxes.conf.item()}")

        #annotate image with bounding box no labels
        self.annotated_frame = self.yolo_results[0].plot(labels=False)

        # Check if any objects are detected and print the message once
        if len(self.yolo_results[0].boxes) == 0 and not self.object_print:
            # No objects detected, return default values
            self.get_logger().info("No objects detected")
            self.label = 'No objects detected'
            self.object_print = True
            self.conf.data = 0.0
            
            return
        elif self.object_print and len(self.yolo_results[0].boxes) == 0:
            return
        
        self.object_print = False
        
        self.conf.data = self.yolo_results[0].boxes.conf.item()

        
        #get class id of identified object
        cls_id = self.yolo_results[0].boxes.cls
        #get the name of the class id
        self.label = self.yolo_results[0].names[int(cls_id)]
        

        #get the coordinates of the bounding box [top_left:row col , bottom_right: row col]
        tb3_box_TLBR = self.yolo_results[0].boxes.xyxy.reshape(-1)
        self.bounding_box.data = np.array(tb3_box_TLBR,dtype=np.int16).tolist()
        
        
        #middle point of x/column
        x = (tb3_box_TLBR[0]+tb3_box_TLBR[2])/2
        #lower point of y, because the base doesnt stretch
        y = tb3_box_TLBR[3]
        #some prep to publish pose, not optimized
        self.pose_xy_pixels = np.array([x,y],dtype=np.int16).tolist()



    def homographic(self,pose_ori):

        pose = np.append(pose_ori,1)

        pose_transformed = self.M.dot(np.int16(pose))
        x_pix = np.int16(pose_transformed[0]/pose_transformed[2])
        y_pix = np.int16(pose_transformed[1]/pose_transformed[2])

        self.pose_xy_pixels_homo.data = np.array([x_pix,y_pix]).tolist()

        pose = PoseStamped()
        pose.header.frame_id = f"map_{self.name}"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.pose_xy_pixels_homo.data[0]*self.homo_resolution
        pose.pose.position.y = self.pose_xy_pixels_homo.data[1]*self.homo_resolution
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        transform = self.tf_buffer.lookup_transform('map',f"map_{self.name}",rclpy.time.Time())
        self.pose_xy_homo = do_transform_pose_stamped(pose,transform)

        
        if self.show_homographic_region:
            pts1 = np.array([self.pts1[0],self.pts1[2],self.pts1[3],self.pts1[1]],np.int32)
            pts1 = pts1.reshape((-1, 1, 2))
            # homo_transform = cv.warpPerspective(self.current_frame,self.M,(1112,1168))
            cv.circle(self.annotated_frame,pose_ori,5,(255,0,0),-1,cv.LINE_4)
            cv.cvtColor(self.annotated_frame,cv.COLOR_RGB2BGR,self.annotated_frame)
            self.annotated_frame = cv.polylines(self.annotated_frame,[pts1],1,[0,0,255],2,cv.LINE_4)
            cv.imshow('image',self.annotated_frame)
            cv.waitKey(1)
        if self.record:
            # self.get_logger().info('Recording')
            self.out.write(self.annotated_frame)
    
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