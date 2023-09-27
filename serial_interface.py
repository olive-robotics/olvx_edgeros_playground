import rclpy
from rclpy.node import Node
import serial

class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')

        # Set up Serial port
        self.serial_port = serial.Serial('/dev/ttyS0', 115200, timeout=1)

        # Create a timer that reads from the serial port every 0.1 seconds
        self.timer = self.create_timer(0.1, self.read_from_serial)

    def read_from_serial(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').rstrip()
            self.get_logger().info('Read: "%s"' % line)

    def write_to_serial(self, message):
        self.serial_port.write(message.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)

    serial_node = SerialNode()

    rclpy.spin(serial_node)

    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

