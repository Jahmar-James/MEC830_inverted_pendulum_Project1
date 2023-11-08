import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import math
import time

class ForceNode(Node):
    def __init__(self, target_link):
        super().__init__('force_node')
        
        if target_link == "cart":
            self.publisher_ = self.create_publisher(Wrench, 'cart_force', 10)
        elif target_link == "pendulum_mass":
            self.publisher_ = self.create_publisher(Wrench, 'pendulum_disturbance_force', 10)
        else:
            raise ValueError("Invalid target link specified")
        
        # Gather user input and apply force accordingly
        self.choose_force_type()

    def choose_force_type(self):
        while True:
                    print("\nChoose the type of force:")
                    print("1: Constant force")
                    print("2: Impulse")
                    print("3: Sine wave")
                    print("4: Exit")
                    choice = input()

                    if choice == "1":
                        magnitude = float(input("Enter the magnitude of the constant force: "))
                        self.apply_constant_force(magnitude)
                    elif choice == "2":
                        magnitude = float(input("Enter the magnitude of the impulse: "))
                        duration = float(input("Enter the duration of the impulse (in seconds): "))
                        self.apply_impulse(magnitude, duration)
                    elif choice == "3":
                        amplitude = float(input("Enter the amplitude of the sine wave: "))
                        frequency = float(input("Enter the frequency of the sine wave: "))
                        duration = float(input("Enter the duration for which sine wave should be applied (in seconds): "))
                        self.apply_sine_wave(amplitude, frequency, duration)
                    elif choice == "4":
                        print("Exiting.")
                        break
                    else:
                        print("Invalid choice!")

    def apply_constant_force(self, magnitude):
        msg = Wrench()
        msg.force.x = magnitude
        self.publisher_.publish(msg)

    def apply_impulse(self, magnitude, duration):
        msg = Wrench()
        msg.force.x = magnitude
        self.publisher_.publish(msg)
        time.sleep(duration)
        msg.force.x = 0.0
        self.publisher_.publish(msg)

    def apply_sine_wave(self, amplitude, frequency, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = Wrench()
            msg.force.x = amplitude * math.sin(2 * math.pi * frequency * (time.time() - start_time))
            self.publisher_.publish(msg)
            time.sleep(0.01)  # sleep for 10ms for smoother transitions

def main(args=None):
    rclpy.init(args=args)
    force_node = ForceNode("cart")  # You can change this to "pendulum_mass" for the pendulum
    rclpy.spin(force_node)

    force_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
