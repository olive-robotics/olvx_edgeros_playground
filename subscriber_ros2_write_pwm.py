import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PWMSubscriber(Node):

  def __init__(self):
    super().__init__('pwm_subscriber')
    
    # Topic to File mapping
    self.topic_to_file = {
      '/olive/edgeros/pwm0': '/sys/class/pwm/pwmchip0/pwm0/duty_cycle',
      '/olive/edgeros/pwm1': '/sys/class/pwm/pwmchip0/pwm1/duty_cycle',
      '/olive/edgeros/pwm2': '/sys/class/pwm/pwmchip0/pwm2/duty_cycle',
      '/olive/edgeros/pwm3': '/sys/class/pwm/pwmchip0/pwm3/duty_cycle',
      '/olive/edgeros/pwm4': '/sys/class/pwm/pwmchip4/pwm0/duty_cycle',
      '/olive/edgeros/pwm5': '/sys/class/pwm/pwmchip4/pwm1/duty_cycle',
      '/olive/edgeros/pwm6': '/sys/class/pwm/pwmchip4/pwm2/duty_cycle',
      '/olive/edgeros/pwm7': '/sys/class/pwm/pwmchip4/pwm3/duty_cycle'
    }
    
    # Read max values from period files and store
    self.max_values = {}
    for topic, duty_cycle_file in self.topic_to_file.items():
      period_file = duty_cycle_file.replace("duty_cycle", "period")
      with open(period_file, 'r') as f:
        self.max_values[topic] = int(f.read().strip())
    
    # Subscribe to topics
    for topic in self.topic_to_file.keys():
      self.create_subscription(Float32, topic, lambda msg, topic=topic: self.pwm_callback(msg, topic), 10)

  def pwm_callback(self, msg, topic):
    value = msg.data
    if 0 <= value <= 1:
      max_value = self.max_values[topic]
      mapped_value = int(value * max_value)
      file_path = self.topic_to_file[topic]
      self.write_to_pwm_file(file_path, mapped_value)
    else:
      self.get_logger().warn(f"Received invalid value {value} for {topic}. Ignoring.")

  def write_to_pwm_file(self, file_path, value):
    try:
      with open(file_path, 'w') as f:
        f.write(str(value))
      self.get_logger().info(f"Written {value} to {file_path}")
    except Exception as e:
      self.get_logger().error(f"Failed to write to {file_path}: {e}")

def main(args=None):
  rclpy.init(args=args)
  pwm_subscriber = PWMSubscriber()
  rclpy.spin(pwm_subscriber)
  pwm_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
