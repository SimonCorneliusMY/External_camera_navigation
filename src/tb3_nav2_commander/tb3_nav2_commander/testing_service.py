
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn



class LifecycleTest(LifecycleNode):
    
    def __init__(self):
        super().__init__('my_lifecycle')
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'")
        return TransitionCallbackReturn.SUCCESS
        
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'cleanup'")
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

def main():

    
    rclpy.init()

    lifecycle = LifecycleTest()

    

    rclpy.spin(lifecycle)

    rclpy.shutdown()


if __name__ == '__main__':
    main()