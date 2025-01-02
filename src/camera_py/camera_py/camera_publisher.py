import rclpy.qos
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from std_msgs.msg import Int32

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

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                            durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                            depth = 10
                            )

        self.fps = FPSCounter()
        #param
        self.declare_parameter('view_camera',False)
        self.view_cam_feed = self.get_parameter('view_camera').get_parameter_value().bool_value
        # Create a publisher for Image messages
        self.publisher_ = self.create_publisher(Image, 'camera/image', qos_policy)
        self.fps_publisher = self.create_publisher(Int32, 'camera/fps', 10)
        # Create a timer to publish images at 30Hz
        self.timer = self.create_timer(0.033, self.timer_callback)  
        self.bridge = CvBridge()
        self.first_publish = True

        if self.view_cam_feed:
            cv2.namedWindow('camera',cv2.WINDOW_NORMAL)
        
        # Open the camera
        self.cap = cv2.VideoCapture(0)  # 0 for the default camera
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) #set the webcam to use MJPG format
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera.")
            exit()

    def timer_callback(self):
        # Read a frame from the camera
       

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image.")
            return
        if self.view_cam_feed:
            cv2.imshow('camera',frame)
            cv2.waitKey(1)
        # Convert the OpenCV image (BGR) to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # Publish the image
        self.publisher_.publish(ros_image)
        # Print the message only once
        if self.first_publish:
            self.get_logger().info('Publishing image...')
            self.first_publish = False
        # Publish fps data
        data = self.fps.fps()
        if  data == None:
            return
        msg = Int32()
        msg.data = data
        self.fps_publisher.publish(msg)


    def destroy_node(self):
        # Release the camera when the node is destroyed
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Camera shutdown')
        node.destroy_node() #automatically shutdown rclpy


if __name__ == '__main__':
    main()