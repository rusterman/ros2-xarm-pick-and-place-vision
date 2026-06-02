import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.pub = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.base_temp = 20.0

    def publish(self):
        msg = Float32()
        msg.data = self.base_temp + random.uniform(-2.0, 5.0)
        self.pub.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data:.2f} C')


def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
