import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Float32

class MinimalSubscriberPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber_publisher')
        # Subscriber setup
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/olive/eio/id01/adc',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Publisher setup
        self.publisher_ = self.create_publisher(Float32, '/olive/eio/id01/pwm7', 10)

    def listener_callback(self, msg):
        # Log the received data
        self.get_logger().info('Received: "%s"' % str(msg.data))
        if msg.data:
            # Publish the first element of the array as a Float32
            pwm_message = Float32()
            pwm_message.data = msg.data[0]
            self.publisher_.publish(pwm_message)
            self.get_logger().info('Publishing to /olive/eio/id01/pwm7: %f' % pwm_message.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber_publisher = MinimalSubscriberPublisher()

    rclpy.spin(minimal_subscriber_publisher)

    # Destroy the node explicitly
    minimal_subscriber_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
