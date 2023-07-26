import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

class ButtonNode(Node):
    def __init__(self):
        super().__init__('button_node')

        # Pin Setup:
        self.button_pin = BUTTON_GPIO_PIN  # Change this to your actual GPIO pin
        GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
        GPIO.setup(self.button_pin, GPIO.IN)  # Button pin set as input

        # TODO: Define your ROS2 subscriptions, services, etc. here

        self.create_timer(0.1, self.read_button_status)  # 10 Hz timer

    def read_button_status(self):
        status = GPIO.input(self.button_pin)
        self.get_logger().info(f'Button status: {"Pressed" if status else "Released"}')

        # TODO: Publish this status on a topic, or respond to service requests with this status

    def shutdown(self):
        GPIO.cleanup()  # cleanup all GPIO

def main(args=None):
    rclpy.init(args=args)

    node = ButtonNode()

    rclpy.spin(node)

    node.shutdown()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

