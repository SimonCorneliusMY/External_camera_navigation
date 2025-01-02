import rclpy
import rclpy.qos
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        policy_best_effort = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                            durability = rclpy.qos.DurabilityPolicy.VOLATILE,
                            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                            depth = 10
                            )
        # Create a CvBridge instance to convert ROS images to OpenCV
        self.bridge = CvBridge()


        # Subscriber for the image topic, replace 'camera/image' with your topic name
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',  # Adjust to your camera topic
            self.listener_callback,
            policy_best_effort
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV2 format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the image using OpenCV
            cv2.imshow("Camera Feed", cv_image)

            cv2.waitKey(1)  # Required to update the window and handle events
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        cv2.destroyAllWindows()  # Close OpenCV windows


if __name__ == '__main__':
    main()
