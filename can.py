import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import struct

CAN_INTERFACE = 'can0'

class CANReadPublisher(Node):
    
    def __init__(self):
        super().__init__('can_read_publisher')
        self.publisher = self.create_publisher(String, 'can_receive_topic', 10)
        self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.can_socket.bind((CAN_INTERFACE,))
        timer_period = 0.1  # 100ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        can_packet = self.can_socket.recv(16)
        can_id, can_data = struct.unpack("<I8s", can_packet)
        message = f"{can_id:X}#{can_data.hex().upper()}"
        msg = String(data=message)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    can_read_publisher = CANReadPublisher()
    rclpy.spin(can_read_publisher)
    can_read_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
