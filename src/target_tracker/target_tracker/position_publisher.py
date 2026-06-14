import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point


class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_publisher')

        self.publisher = self.create_publisher(
            Point,
            'target_position',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.t = 0.0

    def timer_callback(self):

        msg = Point()

        self.t += 0.1

        radius = 10.0

        msg.x = radius * math.cos(self.t)
        msg.y = radius * math.sin(self.t)
        msg.z = 10.0

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Published: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})"
         )


def main(args=None):

    rclpy.init(args=args)

    node = PositionPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
