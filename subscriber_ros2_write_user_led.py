"""
MIT License

Copyright (c) 2023 Olive Robotics GmbH

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

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
