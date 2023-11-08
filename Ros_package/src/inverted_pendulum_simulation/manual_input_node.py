import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

class ManualInputNode(Node):
    def __init__(self):
        super().__init__('manual_input_node')
        self.publisher = self.create_publisher(Wrench, 'cart_force', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Get user input for force to apply
        try:
            force = float(input("Enter force to apply to the cart: "))
            wrench_msg = Wrench()
            wrench_msg.force.x = force
            self.publisher.publish(wrench_msg)
        except ValueError:
            self.get_logger().warn("Please enter a valid number.")

def main(args=None):
    rclpy.init(args=args)
    node = ManualInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
