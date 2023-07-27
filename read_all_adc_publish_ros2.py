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
