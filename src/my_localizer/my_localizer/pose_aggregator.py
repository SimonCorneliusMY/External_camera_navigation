import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import numpy as np

class Pose_Aggregator(Node):
    def __init__(self):
        super().__init__('Pose_Aggregator')

        self.pose = PoseStamped()
        self.pose.header.frame_id = 'Turtlebot3-sim'
        self.pose_count = 0
        self.pose_subscribers = []

        num_cameras = 2  # Example: assume 2 cameras, change as needed
        self.position_x = np.zeros(num_cameras, float)
        self.position_y = np.zeros(num_cameras, float)
        self.position_z = np.zeros(num_cameras, float)

        self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.timer = self.create_timer(1.0, self.calculate_average_pose)
        for i in range(num_cameras):
            topic_name = f'pose_{i}'  # Replace with your actual topic names
            self.pose_subscribers.append(self.create_subscription(
                PoseStamped,  # The message type
                topic_name,
                self.create_pose_callback(i),  # Dynamically generated callback
                10  # QoS value
            ))

    def imu_callback(self, msg: Imu):
        try:
            self.pose.pose.orientation = msg.orientation
        except Exception as e:
            self.get_logger().error(f"Error in imu_callback: {e}")

    def create_pose_callback(self, camera_index):
        """Dynamically generate a callback function for each camera pose."""
        def callback(msg: PoseStamped):
            try:
                # If TurtleBot3 is not detected, set the position to 0
                self.position_x[camera_index] = 0.0
                self.position_y[camera_index] = 0.0
                self.position_z[camera_index] = 0.0

                # If TurtleBot3 is detected, set the position to the detected position
                if msg.header.frame_id == 'Turtlebot3-sim':
                    # Accumulate position (translation)
                    self.position_x[camera_index] = msg.pose.position.x
                    self.position_y[camera_index] = msg.pose.position.y
                    self.position_z[camera_index] = msg.pose.position.z
            except IndexError as e:
                self.get_logger().error(f"IndexError in create_pose_callback: {e}")
            except Exception as e:
                self.get_logger().error(f"Error in create_pose_callback: {e}")
        return callback

    def calculate_average_pose(self):
        try:
            count = np.count_nonzero(self.position_x)
            if count == 0:
                self.get_logger().warn("No valid positions to average.")
                return

            self.pose.header.stamp = self.get_clock().now().to_msg()
            self.pose.pose.position.x = self.position_x.sum() / count
            self.pose.pose.position.y = self.position_y.sum() / count
            self.pose.pose.position.z = self.position_z.sum() / count

            self.pose_publisher.publish(self.pose)

            # Reset for the next set of poses
            self.position_x.fill(0)
            self.position_y.fill(0)
            self.position_z.fill(0)

        except ZeroDivisionError as e:
            self.get_logger().error(f"ZeroDivisionError in calculate_average_pose: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in calculate_average_pose: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Pose_Aggregator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e:
        node.get_logger().error(f"Error in main: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()