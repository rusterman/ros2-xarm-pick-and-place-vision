import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

THRESHOLD = 23.0


class AlertSubscriber(Node):
    def __init__(self):
        super().__init__('alert_subscriber')
        self.sub = self.create_subscription(Float32, 'temperature', self.callback, 10)

    def callback(self, msg):
        temp = msg.data
        if temp > THRESHOLD:
            self.get_logger().warn(f'ALERT: {temp:.2f} C exceeds threshold ({THRESHOLD} C)')
        else:
            self.get_logger().info(f'OK: {temp:.2f} C')


def main(args=None):
    rclpy.init(args=args)
    node = AlertSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
