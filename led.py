import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

class LedNode(Node):

    def __init__(self):
        super().__init__('led_node')
        self.subscription = self.create_subscription(
            Bool,
            'bool_topic',
            self.listener_callback,
            10)

        # Setup GPIO
        self.led_pin = 18  # Set to your LED pin number
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT)

    def listener_callback(self, msg):
        # Use received data to write to LED
        self.write_to_led(msg.data)

    def write_to_led(self, data):
        # Write to LED
        GPIO.output(self.led_pin, data)
        self.get_logger().info('Setting LED state to: "%s"' % data)

def main(args=None):
    rclpy.init(args=args)

    led_node = LedNode()

    rclpy.spin(led_node)

    led_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

