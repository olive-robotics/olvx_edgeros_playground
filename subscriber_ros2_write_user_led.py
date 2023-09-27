import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

LED_USER_GPIO = '/sys/class/gpio/gpiochip9/gpio7/value'

class LEDUserSubscriber(Node):
    
    def __init__(self):
        super().__init__('led_user_subscriber')
        self.subscription = self.create_subscription(
            Bool, 'led_user_command', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        if msg.data:
            with open(LED_USER_GPIO, 'w') as gpio:
                gpio.write('1')
        else:
            with open(LED_USER_GPIO, 'w') as gpio:
                gpio.write('0')

def main(args=None):
    rclpy.init(args=args)
    led_user_subscriber = LEDUserSubscriber()
    rclpy.spin(led_user_subscriber)
    led_user_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
