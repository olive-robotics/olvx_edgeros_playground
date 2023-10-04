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
from std_msgs.msg import String
import socket
import struct

CAN_INTERFACE = 'can0'

class CANWriteSubscriber(Node):
    
    def __init__(self):
        super().__init__('can_write_subscriber')
        self.subscription = self.create_subscription(
            String, 'can_send_topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.can_socket.bind((CAN_INTERFACE,))
        
    def listener_callback(self, msg):
        # Here, we'll assume the received string is a hexadecimal representation of the CAN frame.
        can_id, can_data = self.parse_can_message(msg.data)
        can_packet = struct.pack("<I8s", can_id, can_data)
        self.can_socket.send(can_packet)
        
    def parse_can_message(self, message):
        # Simple example to parse a message like '5A1#AABBCCDD'
        parts = message.split('#')
        can_id = int(parts[0], 16)
        can_data = bytes.fromhex(parts[1])
        return can_id, can_data

def main(args=None):
    rclpy.init(args=args)
    can_write_subscriber = CANWriteSubscriber()
    rclpy.spin(can_write_subscriber)
    can_write_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
