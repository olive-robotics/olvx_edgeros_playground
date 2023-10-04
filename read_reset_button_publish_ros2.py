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
