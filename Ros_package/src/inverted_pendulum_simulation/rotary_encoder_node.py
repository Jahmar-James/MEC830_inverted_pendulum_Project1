import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RotaryEncoderNode(Node):

    def __init__(self):
        super().__init__('rotary_encoder_node')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        # Assuming one encoder, initialize count to zero
        self.encoder_count = 0
        # Conversion factor: how many encoder counts correspond to one radian of rotation.
        self.counts_per_radian = 1000  

    def listener_callback(self, msg: JointState):
        # Find the index of the joint of interest in the message
        joint_name = "cart_pendulum_rod_joint"
        try:
            index = msg.name.index(joint_name)
        except ValueError:
            self.get_logger().warn(f"{joint_name} not found in JointState message.")
            return
        
        # Get the joint angle in radians
        joint_angle = msg.position[index]

        # Convert joint angle to encoder counts
        self.encoder_count = int(joint_angle * self.counts_per_radian)

        # Now, self.encoder_count holds the computed encoder count
        self.get_logger().info(f"Encoder count for {joint_name}: {self.encoder_count}")

def main(args=None):
    rclpy.init(args=args)
    rotary_encoder_node = RotaryEncoderNode()
    rclpy.spin(rotary_encoder_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
