import rclpy
import rclpy.qos
import cv2 as cv
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class localizer(Node):

    def __init__(self):
        super().__init__('localizer')
        self.br = CvBridge()
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                    durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth = 10
                                    )
        self.pose = Pose()
        self.pose_pixel = Int16MultiArray()

        self.pose_tf_publisher = TransformBroadcaster(self,10)
        self.camera_subscription = self.create_subscription(Image,'camera/image_raw', self.camera_callback,10)
        self.imu_subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.pose_publisher = self.create_publisher(Pose,'pose',qos_policy)
        self.pose_pixel_publisher = self.create_publisher(Int16MultiArray,'pose_pixel',qos_policy)
        self.pose_timer = self.create_timer(0.033,self.pose_timer_callback)
        

    def imu_callback(self,data:Imu):
        try:

            self.pose.orientation.x = data.orientation.x
            self.pose.orientation.y = data.orientation.y
            self.pose.orientation.z = data.orientation.z
            self.pose.orientation.w = data.orientation.w

        except Exception as e:
            print(e)

        finally:
            return
    
    def camera_callback(self,data:Image):
        
        try:

            image = self.br.imgmsg_to_cv2(data)
            self.colour_localization_mapping(image)



        except Exception as e:
            print(e)

        finally:
            return
        
    def pose_timer_callback(self):
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
        odom_to_basefootprint.transform.translation.x = self.pose.position.x
        odom_to_basefootprint.transform.translation.y = self.pose.position.y #+1080*4/1044   #1080*4/1044 added because global costmap not same as map 24/10/2024
        odom_to_basefootprint.transform.translation.z = 0.0
        

        odom_to_basefootprint.transform.rotation.x = self.pose.orientation.x
        odom_to_basefootprint.transform.rotation.y = self.pose.orientation.y
        odom_to_basefootprint.transform.rotation.z = self.pose.orientation.z
        odom_to_basefootprint.transform.rotation.w = self.pose.orientation.w

        
        self.pose_tf_publisher.sendTransform(odom_to_basefootprint)
        self.pose_tf_publisher.sendTransform(map_to_odom)
        self.pose_publisher.publish(self.pose)
        self.pose_pixel_publisher.publish(self.pose_pixel)


    def colour_localization_mapping(self,current_frame:np.array):
        default_pose = np.array([1.0,1.0])
        default_maze = np.zeros((10,10),np.uint8)
        if len(np.shape(current_frame)) != 3: #check if image data is coming in
            return default_pose, default_maze #just a arbitrary default value for x and y
        
        maze_hsv = cv.cvtColor(current_frame,cv.COLOR_BGR2HSV)   #hue (0 - 179), saturation and value (0 - 255)
        #define upper and lower hsv value to isolate
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
        lower_green = np.array([50,50,50])
        upper_green = np.array([70,255,255])


        #create mask(isolate colour of interest)
        mask_blue = cv.inRange(maze_hsv,lower_blue,upper_blue)
        mask_green = cv.inRange(maze_hsv,lower_green,upper_green)


        # self.get_logger().info('Blue: {}  Green: "{}"'.format(object_counter(mask_blue),object_counter(mask_green)) )
        #check for green and blue localization
        if object_counter(mask_blue) != 0 and object_counter(mask_green) != 0:

            #get the index of the blue and green pixels
            blue_location = np.nonzero(mask_blue)
            green_location = np.nonzero(mask_green)

            #identify the top left corner and bottom right corner coordinates,creating a rectangle bounding box
            green_location_min= np.min([green_location[0],green_location[1]],axis=1)
            green_location_max = np.max([green_location[0],green_location[1]],axis=1)
            #green_minmax = (green_location_min,green_location_max)

            blue_location_min= np.min([blue_location[0],blue_location[1]],axis=1)
            blue_location_max = np.max([blue_location[0],blue_location[1]],axis=1)
            #blue_minmax = (blue_location_min,blue_location_max)

            #obtain the midpoint coordinate of the rectangle
            blue_rowcol = np.int32((blue_location_max + blue_location_min)/2)
            green_rowcol = np.int32((green_location_max + green_location_min)/2) 

            #shift row starting point for top left to bottom left
            self.pose_xy_pixel_TL = [current_frame.shape[0] - blue_rowcol[0],blue_rowcol[1]]

            self.pose.position.x = blue_rowcol[1]*4/1044 
            self.pose.position.y = (current_frame.shape[0] - blue_rowcol[0])*4/1044

            self.pose_pixel.data = blue_rowcol[::-1].tolist()




            return   #4m = 1044 pixels, blue_rowcol is in pixels
        
        return 

def object_counter(bw_image):

    (contours, hierarchy) = cv.findContours(
        bw_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    return len(contours)


def main(args=None):
    rclpy.init(args=args)
    node = localizer()
    rclpy.spin(node)

    node.destroy_node()

if __name__== '__main__':
    main()