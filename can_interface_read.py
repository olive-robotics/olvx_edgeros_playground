import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import struct

CAN_INTERFACE = 'can0'

## \file
#  \brief A file for reading and publishing CAN bus messages using ROS 2
#
#  This file contains a ROS 2 node that reads messages from a CAN interface
#  and publishes them to a ROS 2 topic.

class CANReadPublisher(Node):
    
    ## \class CANReadPublisher
    #  \brief A class for reading CAN bus messages and publishing them using ROS 2.
    #
    #  This class creates a ROS 2 node that reads messages from a specified CAN interface
    #  and publishes them to a specified ROS 2 topic in a human-readable format.
    
    def __init__(self):
        ## \brief Constructor of the CANReadPublisher class.
        #
        #  Initializes the node, creates a publisher, opens a socket to the CAN interface,
        #  and creates a timer to read messages from the CAN bus at regular intervals.
        
        super().__init__('can_read_publisher')
        self.publisher = self.create_publisher(String, 'can_receive_topic', 10)
        self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.can_socket.bind((CAN_INTERFACE,))
        timer_period = 0.1  # 100ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        ## \brief Timer callback to read and publish CAN messages.
        #
        #  Reads a message from the CAN bus, formats it, and publishes it
        #  to a ROS 2 topic.
        
        can_packet = self.can_socket.recv(16)
        can_id, can_data = struct.unpack("<I8s", can_packet)
        message = f"{can_id:X}#{can_data.hex().upper()}"
        msg = String(data=message)
        self.publisher.publish(msg)

def main(args=None):
    ## \brief The main function to execute the CANReadPublisher node.
    #
    #  Initializes ROS 2, creates an instance of CANReadPublisher, spins the node to keep it running,
    #  and then cleans up the node and shuts down ROS 2 when it's done.
    
    rclpy.init(args=args)
    can_read_publisher = CANReadPublisher()
    rclpy.spin(can_read_publisher)
    can_read_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
