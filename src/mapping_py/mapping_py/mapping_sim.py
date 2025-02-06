import rclpy
import rclpy.qos
import cv2 as cv
import numpy as np
import time

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int16MultiArray

"""
Unresolved issue of slow (0.15sec/6fps) at assigning large data to OccupancyGrid switch to using cpp for mapping 24/01/13
"""
class FPSCounter:
    def __init__(self):
        self.frame_count = 0
        self.start_time = time.time()

    def fps(self):
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time

        if elapsed_time >3.0:
            fps = self.frame_count/elapsed_time
            self.frame_count = 0
            self.start_time = time.time()
            return int(fps)
        return 

class Mapping(Node):

    def __init__(self):
        super().__init__('mapping')
        self.declare_parameter('save_map',False)
        self.save_map = self.get_parameter('save_map').get_parameter_value().bool_value
        qos_profile_reliable = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                            durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                            depth = 10
                            )
        self.bounding_box = self.create_subscription(Int16MultiArray, 'pose_pixel', self.bounding_box_callback, qos_profile_reliable)
        self.image = self.create_subscription(Image,'camera/image_raw', self.image_callback,10)

        self.map_publisher = self.create_publisher(OccupancyGrid,'map',qos_profile_reliable)
        self.map = OccupancyGrid()
        self.br = CvBridge()
        self.fps = FPSCounter()
        # self.pose_xy = list()

    def image_callback(self,data):
        try:

            self.mapping(self.br.imgmsg_to_cv2(data),self.pose_pixel,80)

        except Exception as e:
            print(e)

        finally:
            return
    
    def bounding_box_callback(self,data:Int16MultiArray):
        self.pose_pixel = data.data
        
        return    
    
    def mapping(self,image:np.ndarray,point: list[int], size: int):
        #threshold values to isolate floor TODO: pass in as parameter
        print(self.fps.fps())

        lower_gray = np.array([10,0,155])
        lower_mid_gray = np.array([0,0,155])
        upper_mid_gray = np.array([180,0,155])
        upper_gray = np.array([170,0,155])

        #isolating
        hsv = cv.cvtColor(image,cv.COLOR_BGR2HSV)
        mask_lower = cv.inRange(hsv,lower_mid_gray,lower_gray)
        mask_upper = cv.inRange(hsv,upper_mid_gray,upper_gray)

        mask = mask_lower | mask_upper  # Use bitwise OR for combining masks

        # Define the square region around the given point
        half_size = size // 2

        start_x = max(0, point[1] - half_size)
        end_x = min(mask.shape[0] - 1, point[1] + half_size)
        start_y = max(0, point[0] - half_size)
        end_y = min(mask.shape[1] - 1, point[0] + half_size)

        
        # Set values inside the square to 255 (indicating free space)
        mask[start_x:end_x+1, start_y:end_y+1] = 255

        # Apply morphological opening (erosion followed by dilation)
        kernel = np.ones((11, 11), np.uint8)
        mask_open = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

        # Set map values for obstacles and free space
        mask_open = np.where(mask_open == 0, 100, 0)  # Set obstacle = 100, free path = 0

        # Flip the mask (ROS map data has the origin at bottom-left, while OpenCV is top-left)
        maze_bw_flip = np.flipud(mask_open)


        # Update the map data and publish
        self.map.header.frame_id = 'map'
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map.info.height = np.shape(mask_open)[0]
        self.map.info.width = np.shape(mask_open)[1]
        self.map.info.resolution = 4/1044    #4m = 1044 pixels
        #map's pose
        
        self.map.info.origin.orientation.x = 0.0
        self.map.info.origin.orientation.y = 0.0
        self.map.info.origin.orientation.z = 0.0
        self.map.info.origin.orientation.w = 1.0
        self.map.info.origin.position.x =  0.0 #manually obtained through trial and error, will fix later 7/10/2024
        self.map.info.origin.position.y = 0.0    #1080*4/1044
        self.map.info.origin.position.z = 0.0   
        #TODO slow line below at 0.15 seconds

        if self.save_map == True:
            cv.imwrite('/home/tarumt2204/External_camera_navigation/maze_100.pgm',mask_open)
            print('Image saved')
            self.save_map = False


        self.map.data = maze_bw_flip.flatten().astype(np.int8).tolist()  # Flatten to 1D and convert to int8


        self.map_publisher.publish(self.map)
        return 


def main(args=None):
    rclpy.init(args=args)
    map_node = Mapping()
    rclpy.spin(map_node)

    map_node.destroy_node()

if __name__== '__main__':
    main()