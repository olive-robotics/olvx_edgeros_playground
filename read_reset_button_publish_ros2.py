import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

RESET_BUTTON_GPIO = '/sys/class/gpio/gpiochip7/gpio8/value'

class ResetButtonPublisher(Node):
    
    def __init__(self):
        super().__init__('reset_button_publisher')
        self.publisher = self.create_publisher(Bool, 'reset_button_status', 10)
        timer_period = 0.1  # 100ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        with open(RESET_BUTTON_GPIO, 'r') as gpio:
            value = gpio.read().strip()
        msg = Bool(data=bool(int(value)))
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    reset_button_publisher = ResetButtonPublisher()
    rclpy.spin(reset_button_publisher)
    reset_button_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
