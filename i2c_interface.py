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
from smbus2 import SMBus, i2c_msg

class I2CNode(Node):
    def __init__(self):
        super().__init__('i2c_node')

        self.bus = SMBus(1)
        self.address = 0x20

        # TODO: Define your ROS2 subscriptions, services, etc. here

    def read_i2c(self, register, length):
        msg = i2c_msg.read(self.address, length)
        self.bus.i2c_rdwr(msg)
        return list(msg)

    def write_i2c(self, register, data):
        msg = i2c_msg.write(self.address, [register] + data)
        self.bus.i2c_rdwr(msg)

    def shutdown(self):
        self.bus.close()

def main(args=None):
    rclpy.init(args=args)

    node = I2CNode()

    rclpy.spin(node)

    node.shutdown()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

