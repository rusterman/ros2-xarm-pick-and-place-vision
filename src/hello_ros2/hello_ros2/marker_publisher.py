import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'hello_ros2'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 1.0

        # disappears after 2 s if publisher stops — keeps Foxglove clean
        marker.lifetime.sec = 2

        self.pub.publish(marker)
        self.get_logger().info('Marker published at (0, 0, 0.5)')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
