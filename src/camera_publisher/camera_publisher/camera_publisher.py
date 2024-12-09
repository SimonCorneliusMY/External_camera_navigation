import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # Create a publisher for Image messages
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        # Create a timer to publish images at 10 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)  # 10Hz
        self.bridge = CvBridge()
        self.first_publish = True
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