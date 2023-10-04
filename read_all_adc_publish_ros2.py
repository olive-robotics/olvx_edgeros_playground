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
from std_msgs.msg import Int32MultiArray
import time

class VoltageReader(Node):

    def __init__(self):
        super().__init__('voltage_reader')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'voltage_topic', 10)
        self.timer_period = 0.01  # seconds (100 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.prev_time = time.time()

        # List of files to read from
        self.files = [
            '/sys/bus/iio/devices/iio:device0/in_voltage1_raw',
            '/sys/bus/iio/devices/iio:device0/in_voltage2_raw',
            '/sys/bus/iio/devices/iio:device0/in_voltage4_raw',
            '/sys/bus/iio/devices/iio:device0/in_voltage5_raw',
            '/sys/bus/iio/devices/iio:device0/in_voltage6_raw',
            '/sys/bus/iio/devices/iio:device0/in_voltage7_raw',
            '/sys/bus/iio/devices/iio:device0/in_voltage9_raw',
            '/sys/bus/iio/devices/iio:device0/in_voltage14_raw',
        ]

        self.file_handles = [open(file, 'r') for file in self.files]
        self.log_counter = 0

    def read_file(self, file_handle):
        file_handle.seek(0)
        return int(file_handle.read().strip())

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = [self.read_file(fh) for fh in self.file_handles]
        self.publisher_.publish(msg)
        curr_time = time.time()
        actual_rate = 1.0 / (curr_time - self.prev_time)
        if self.log_counter % 1000 == 0:  # log every 1000 callbacks
            self.get_logger().info('Publishing: "%s", Actual Rate: %.2f Hz' % (str(msg.data), actual_rate))
        self.prev_time = curr_time
        self.log_counter += 1

    def destroy(self):
        for file_handle in self.file_handles:
            file_handle.close()
        super().destroy()


def main(args=None):
    rclpy.init(args=args)

    voltage_reader = VoltageReader()

    try:
        rclpy.spin(voltage_reader)
    except KeyboardInterrupt:
        pass

    voltage_reader.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
