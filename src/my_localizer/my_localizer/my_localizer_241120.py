import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from cv_bridge import CvBridge
import rclpy.publisher
from sensor_msgs.msg import Image

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from turtlebot3_msgs.msg import Astar
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Imu 
from geometry_msgs.msg import PoseWithCovarianceStamped

 

import cv2 as cv
import numpy as np
from heapq import *
import math
import time

import csv
import os
import ctypes





class top_camera(LifecycleNode):

    def __init__(self):
        super().__init__('localizer')

        cv.namedWindow('crop image',cv.WINDOW_NORMAL) 
        cv.namedWindow('flip image',cv.WINDOW_NORMAL) 
        self.current_frame = 0
        self.imu_data= None
        self.br = None
        self.localization_tf = None
        self.timer = None
        self.imu_subscription = None
        self.camera_subscription = None
        np.set_printoptions(suppress=True,precision=2)  #suppress scientific notation and set to 2 decimal places for numpy value.
        self.current_frame = 0
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                    durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth = 10
                                    )
        self.imu_data= Imu()
        self.camera_subscription = self.create_subscription(Image,'camera/image_raw', self.camera_callback,10)

        
        self.br = CvBridge()


        #self.map = StaticTransformBroadcaster(self,qos_policy)
        self.localization_tf = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01,self.broadcast_transform)
        self.imu_subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid,'map',qos_policy)
        self.camera_pose = self.create_publisher(PoseWithCovarianceStamped,'amcl_pose',qos_policy)
        #self.camera_pose_timer = self.create_timer(1.0,self.camera_pose_callback)
        self.map_timer = self.create_timer(0.01,self.map_callback)

 #lifecycle is used to activate simple commander api, normal node failed 27/9/2024  
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'")

        
        # self.imu_data= Imu() 
        # self.br = CvBridge()
        # self.localization_tf = TransformBroadcaster(self)
        # self.timer = self.create_timer(0.01,self.broadcast_transform)
        # self.timer.cancel
        # self.imu_subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        # self.camera_subscription = self.create_subscription(Image,'camera/image_raw', self.camera_callback,10)
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
        pose = self.localization()
        maze_bw = mapping(self.current_frame)
        #maze_bw[maze_bw==255] = 1   #255 is obstacle and change to 1 for costmap

        map.header.frame_id = 'map'
        map.header.stamp = self.get_clock().now().to_msg()
        map.info.height = np.shape(maze_bw)[0]
        map.info.width = np.shape(maze_bw)[1]
        map.info.resolution = 4/1044    #4m = 1044 pixels
        #map's pose
        
        map.info.origin.orientation.x = 0.0
        map.info.origin.orientation.y = 0.0
        map.info.origin.orientation.z = 0.0
        map.info.origin.orientation.w = 1.0
        map.info.origin.position.x =  0.0 #manually obtained through trial and error, will fix later 7/10/2024
        map.info.origin.position.y = 0.0    #1080*4/1044
        map.info.origin.position.z = 0.0          

        maze_bw_flip = cv.flip(maze_bw,0)   #flip horizontally opencv data is origin is top left while map data is bottom left
        maze_bw_flip_list = np.array(maze_bw_flip,np.int8).reshape(-1).tolist()    #row major order, and make sure values are int8

        cv.imshow("crop image",maze_bw)
        cv.imshow("flip image",maze_bw_flip[25:525,25:525])

        map.data = maze_bw_flip_list
        
        self.map_publisher.publish(map)


        odom_to_basefootprint = TransformStamped()
        odom_to_basefootprint.header.stamp = self.get_clock().now().to_msg()
        odom_to_basefootprint.header.frame_id = 'odom'
        odom_to_basefootprint.child_frame_id = 'base_footprint'
        odom_to_basefootprint.transform.translation.x = pose[1]
        odom_to_basefootprint.transform.translation.y = pose[0] #+1080*4/1044   #1080*4/1044 added because global costmap not same as map 24/10/2024
        odom_to_basefootprint.transform.translation.z = 0.0
        

        odom_to_basefootprint.transform.rotation.x = self.imu_data.orientation.x
        odom_to_basefootprint.transform.rotation.y = self.imu_data.orientation.y
        odom_to_basefootprint.transform.rotation.z = self.imu_data.orientation.z
        odom_to_basefootprint.transform.rotation.w = self.imu_data.orientation.w

        
        self.localization_tf.sendTransform(odom_to_basefootprint)

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
        # self.localization_tf.sendTransform(map_to_odom)
        xyz=euler_from_quaternion(self.imu_data.orientation.x,self.imu_data.orientation.y,self.imu_data.orientation.z,self.imu_data.orientation.w)

        #self.get_logger().info('Current pose: {}  orientation: "{}"'.format(pose[::-1],xyz) )

    def camera_callback(self, data):
        self.current_frame = self.br.imgmsg_to_cv2(data)
        # cv.imshow('camera',self.current_frame)
        # cv.waitKey(1)    

    def imu_callback(self,data):
        self.imu_data= data
        
    def broadcast_transform(self):
        # x= 1.0    #meter
        # y = 1.0   #meter
        # theta = 0.5 #rad
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



    def localization(self):
        default_pose = [1.0,1.0]
        if len(np.shape(self.current_frame)) != 3: #check if image data is coming in
            return default_pose #just a arbitrary default value for x and y
        
        maze_hsv = cv.cvtColor(self.current_frame,cv.COLOR_BGR2HSV)   #hue (0 - 179), saturation and value (0 - 255)
        #define upper and lower hsv value to isolate
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
        lower_green = np.array([50,50,50])
        upper_green = np.array([70,255,255])


        #create mask(isolate colour of interest)
        mask_blue = cv.inRange(maze_hsv,lower_blue,upper_blue)
        mask_green = cv.inRange(maze_hsv,lower_green,upper_green)


        self.get_logger().info('Blue: {}  Green: "{}"'.format(object_counter(mask_blue),object_counter(mask_green)) )
        #check for green and blue localization
        if object_counter(mask_blue) != 0 and object_counter(mask_green) != 0:

            #get the index of the blue and green pixels
            blue_location = np.nonzero(mask_blue)
            green_location = np.nonzero(mask_green)

            #identify the top left corner and bottom right corner coordinates,creating a rectangle bounding box
            green_location_min= np.min([green_location[0],green_location[1]],axis=1)
            green_location_max = np.max([green_location[0],green_location[1]],axis=1)
            green_minmax = (green_location_min,green_location_max)

            blue_location_min= np.min([blue_location[0],blue_location[1]],axis=1)
            blue_location_max = np.max([blue_location[0],blue_location[1]],axis=1)
            blue_minmax = (blue_location_min,blue_location_max)

            #obtain the midpoint coordinate of the rectangle
            blue_rowcol = np.int32((blue_location_max + blue_location_min)/2)
            blue_rowcol[0] = self.current_frame.shape[0] - blue_rowcol[0]

            green_rowcol = np.int32((green_location_max + green_location_min)/2) 

            # maze_gray = cv.cvtColor(self.current_frame,cv.COLOR_BGR2GRAY)
            # ret,maze_bw = cv.threshold(maze_gray,125,255,cv.THRESH_BINARY + cv.THRESH_OTSU)
            

            return blue_rowcol*4/1044  #4m = 1044 pixels, blue_rowcol is in pixels
        
        return default_pose


    def service_callback(self,request,response):        #14/6/2024 creating a client service connection between astarservice node and teleop node
        print(request)
        #response = 'Recevived request %d' % response
        
        if request.value == "request A*":
            print(1)
            response = astarservice(response)
            return response
        print(2)
        return "Invalid"
    
    def motionsubscription_callback(self,data):
        print(data)

def mapping(image):
    #threshold values to isolate floor TODO: pass in as parameter
    lower_gray = np.array([10,0,155])
    lower_mid_gray = np.array([0,0,155])
    upper_mid_gray = np.array([180,0,155])
    upper_gray = np.array([170,0,155])

    #isolating
    hsv = cv.cvtColor(image,cv.COLOR_BGR2HSV)
    mask_lower = cv.inRange(hsv,lower_mid_gray,lower_gray)
    mask_upper = cv.inRange(hsv,upper_mid_gray,upper_gray)
    mask = mask_lower + mask_upper

    return mask

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

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    obstacle_value = 255
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:
        
        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return tuple(data) + tuple([start])

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == obstacle_value:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
                
    return False
#ros2 can't send np array due to data type wrappers, ie  np.array([1,2],np.int32), the numbers 1 & 2 has an int32 data wrapper
def angle360(path):
    xyarray = np.array(path)    #convert to array for maths operations
    angles = np.empty(len(path)-1)  #create empty array to store angles, the number of angles is 1 less than the path
    
    xylength = np.empty(xyarray.shape,np.float32) #create empty array to store angles
    standardized_rad = 1/math.pi #convert 0 ~ pi to 0 ~ 1
    ind =0
    magnitude = []
    angle = []
    coordinate=[]
    test = []
    #quaternion = []
    #coordinate.append(path[-1])    #appends starting position of TurtleBot3
    for i in range(0,len(xyarray)-1):
        xylength[i] = xyarray[-i-2] - xyarray[-i-1] #find the length of x and y
        angles[i] = math.atan2(-xylength[i,0],xylength[i,1]) #find the angle of x,y relative to positive x-axis (+y returns -angle and vice versa)

        
        if angles[i] != angles[i-1] and i>0:    #if current angle not same as previous angle save previous angle in a list
            magnitude.append(i -ind)
            angle.append(((angles[i-1]+2*math.pi)%(2*math.pi))*180/math.pi)
            #angle.append(float(angles[i-1])*standardized_rad) 
            coordinate.append(path[-i-1])
            ind = i
    
    #test.append(float(angles[-1])*standardized_rad)
    angle.append(((angles[-1]+2*math.pi)%(2*math.pi))*180/math.pi)
    angle = np.array(angle,np.int32).tolist()
    coordinate = np.reshape(coordinate,-1) #reshape it to a 1D because I am too lazy to figure how to send a 2D array via ros2. Also remove data wrapper
    return angle, magnitude, coordinate

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

def erase_start_goal():

    return 0

#currently using localization in class above 27/9/2024
def location_old(maze):
    if len(np.shape(maze)) != 3: #check if image data is coming in
        return 1.0,1.0  #just a arbitrary default value for x and y
    response = Astar()

    resolution = 4/1044 #m/pixel

    maze_hsv = cv.cvtColor(maze,cv.COLOR_BGR2HSV)   #hue (0 - 179), saturation and value (0 - 255)
    #define upper and lower hsv value to isolate
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    lower_green = np.array([50,50,50])
    upper_green = np.array([70,255,255])

    #create mask(isolate colour of interest)
    mask_blue = cv.inRange(maze_hsv,lower_blue,upper_blue)
    mask_green = cv.inRange(maze_hsv,lower_green,upper_green)

    #get the index of the blue and green pixels
    blue_location = np.nonzero(mask_blue)
    green_location = np.nonzero(mask_green)

    #identify the top left corner and bottom right corner coordinates,creating a rectangle bounding box
    green_location_min= np.min([green_location[0],green_location[1]],axis=1)
    green_location_max = np.max([green_location[0],green_location[1]],axis=1)
    green_minmax = (green_location_min,green_location_max)

    blue_location_min= np.min([blue_location[0],blue_location[1]],axis=1)
    blue_location_max = np.max([blue_location[0],blue_location[1]],axis=1)
    blue_minmax = (blue_location_min,blue_location_max)

    #obtain the midpoint coordinate of the rectangle
    blue_rowcol = np.int32((blue_location_max + blue_location_min)/2)
    green_rowcol = np.int32((green_location_max + green_location_min)/2)

    maze_gray = cv.cvtColor(maze,cv.COLOR_BGR2GRAY)
    ret,maze_bw = cv.threshold(maze_gray,125,255,cv.THRESH_BINARY + cv.THRESH_OTSU)

    response.coordinate = np.append(blue_rowcol,green_rowcol,0).tolist()

    response.test = np.append(green_location_min,[green_location_max,blue_location_min,blue_location_max]).tolist()

    return response, maze_bw

def localization(current_frame):
    default_map = np.empty([10,10])
    default_pose = [1.0,1.0]
    if len(np.shape(current_frame)) != 3: #check if image data is coming in
        return default_pose,default_map #just a arbitrary default value for x and y
    
    maze_hsv = cv.cvtColor(current_frame,cv.COLOR_BGR2HSV)   #hue (0 - 179), saturation and value (0 - 255)
    #define upper and lower hsv value to isolate
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    lower_green = np.array([50,50,50])
    upper_green = np.array([70,255,255])

    #create mask(isolate colour of interest)
    mask_blue = cv.inRange(maze_hsv,lower_blue,upper_blue)
    mask_green = cv.inRange(maze_hsv,lower_green,upper_green)


    #check for green and blue localization
    if object_counter(mask_blue) != 0 and object_counter(mask_green) != 0:


        #get the index of the blue and green pixels
        blue_location = np.nonzero(mask_blue)
        green_location = np.nonzero(mask_green)

        #identify the top left corner and bottom right corner coordinates,creating a rectangle bounding box
        green_location_min= np.min([green_location[0],green_location[1]],axis=1)
        green_location_max = np.max([green_location[0],green_location[1]],axis=1)
        green_minmax = (green_location_min,green_location_max)

        blue_location_min= np.min([blue_location[0],blue_location[1]],axis=1)
        blue_location_max = np.max([blue_location[0],blue_location[1]],axis=1)
        blue_minmax = (blue_location_min,blue_location_max)

        #obtain the midpoint coordinate of the rectangle
        blue_rowcol = np.int32((blue_location_max + blue_location_min)/2)
        blue_rowcol[0] = current_frame.shape[0] - blue_rowcol[0]

        green_rowcol = np.int32((green_location_max + green_location_min)/2) 

        maze_gray = cv.cvtColor(current_frame,cv.COLOR_BGR2GRAY)
        ret,maze_bw = cv.threshold(maze_gray,125,255,cv.THRESH_BINARY + cv.THRESH_OTSU)
        

        return blue_rowcol*4/1044, maze_bw  #4m = 1044 pixels, blue_rowcol is in pixels
    
    return default_pose,default_map


def astarservice(start,goal,maze,resize_size,test):
    #rclpy.init(args=args)
    start,goal = np.array([start,goal])
    frame_dim = np.shape(maze)

    resize_factor = (1/np.array(frame_dim[0]))*resize_size[1]   #we scale by the column, cv.resize takes (width,height)
    start = tuple(np.array(start*resize_factor,np.int8))

    goal = tuple(np.array(goal*resize_factor,np.int8))
    response =Astar()
    astar_time = []
    #cv.namedWindow('test',cv.WINDOW_NORMAL)
    while 1:

        # saveimg = cv.cvtColor(maze,cv.COLOR_BGR2RGB)
        # cv.imwrite('/home/tarumt2204/maze0711.png',saveimg)

        
        maze_gray = cv.cvtColor(maze,cv.COLOR_BGR2GRAY)
        ret, maze_bw = cv.threshold(maze_gray,127,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
        indices = maze_bw.nonzero()     
        min = np.min(indices,axis=1)
        max = np.max(indices,axis=1)
        #maze_bw_slice = maze_bw[min[0]:max[0],min[1]:max[1]]    #remove the edges of the map that doesnt capture the maze
        maze_30 = cv.resize(maze_bw,resize_size ,interpolation=cv.INTER_NEAREST_EXACT)  #get timing for different sizes 4/7/2024

        maze_30[goal] = 0
        # maze_30[test[0]:test[2],test[1]:test[3]]=0
        # maze_30[test[4]:test[6],test[5]:test[7]]=0
        
        # maze_30_color = np.zeros(np.append(maze_30.shape,3),np.uint8)
        # maze_30_color[maze_30==255] = 255
        # maze_30_color[start] = (0,255,0)
        # maze_30_color[goal] = (255,0,0)
#         cv.imshow('test',maze_30)
#         if cv.waitKey(0) == ord('q'):
#             break
# # When everything done, release the capture

#         cv.destroyAllWindows()


        astar_start = time.time()
        path = astar(maze_30,start,goal)
 
        astar_stop = time.time()
        angle,magnitude,coordinate  = angle360(path)
        
        astar_time.append(astar_stop-astar_start)
        #csv_writer("test.csv","test",astar_time)    #24/5 not tested, save astar time taken for report
        coordinate = np.array(coordinate/resize_factor,np.int32).tolist()
        
        response.angle = angle
        response.magnitude = magnitude
        response.coordinate = coordinate


        return response


def main(args=None):
    rclpy.init(args=args)
    camera = top_camera()
    cv.namedWindow('camera',cv.WINDOW_NORMAL)   #cv.window_normal to allow readjusting of window size

    x = 0
    resize_size = (60,34)
    
    #minimal_subscriber = MinimalSubscriber()
    while 1:

        rclpy.spin_once(camera)

        #rclpy.spin_once(camera)
        #TB3_location = location(camera.current_frame)
        #print(np.array(TB3_location.coordinate[0:2]))
        # if x == 0:
        #     result = astarservice(TB3_location.coordinate[0:2],TB3_location.coordinate[2:4],camera.current_frame,resize_size,TB3_location.test)
        #     print(result)
        #     #coordinates = np.reshape(result.coordinate,(int(len(result.coordinate)/2),2))
        #     x=1
            
        
        # camera.motionpublisher.publish(result)
        # camera.coordinate_publisher.publish(TB3_location)

        # obstacle = (255,255,255)
        # path_planned = (0,255,0)
        # maze_path_plot_euclidean =np.zeros(np.append(maze_30.shape,3),np.uint8)
        # maze_path_plot_euclidean[maze_30==255]=obstacle
        
        # for index in coordinates:
        #     maze_path_plot_euclidean[index[0],index[1]]=path_planned
        #locate, maze_bw = location(camera.current_frame)
        
        cv.resizeWindow('camera',800,800)
        
        cv.imshow('camera',camera.current_frame)
  
        # cv.imshow("Plot maze",maze_path_plot_euclidean)
        cv.waitKey(1)   #0 means it will continue to show the image till a key is pressed hence it wont loop the code, 1 means it waits 1ms then continue the loop

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)



    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()