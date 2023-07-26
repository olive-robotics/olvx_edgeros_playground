import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

class PwmNode(Node):

    def __init__(self):
        super().__init__('pwm_node')
        self.subscription = self.create_subscription(
            Float32,
            'float_topic',
            self.listener_callback,
            10)

        # Setup GPIO
        self.pwm_pin = 18  # Set to your PWM pin number
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 100)  # 100Hz frequency
        self.pwm.start(0)  # Initial duty cycle of 0

    def listener_callback(self, msg):
        # Use received data to write to PWM
        self.write_to_pwm(msg.data)

    def write_to_pwm(self, data):
        # Write to PWM
        self.pwm.ChangeDutyCycle(data)
        self.get_logger().info('Writing to PWM: "%s"' % data)

def main(args=None):
    rclpy.init(args=args)

    pwm_node = PwmNode()

    rclpy.spin(pwm_node)

    pwm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


