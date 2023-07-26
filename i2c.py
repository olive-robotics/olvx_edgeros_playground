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

