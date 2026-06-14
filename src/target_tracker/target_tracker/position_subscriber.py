import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class PositionSubscriber(Node):

    def __init__(self):
        super().__init__('position_subscriber')

        self.subscription = self.create_subscription(
            Point,
            'target_position',
            self.callback,
            10
        )

    def callback(self, msg):

        self.get_logger().info(
            f"Target: X={msg.x:.1f}, Y={msg.y:.1f}, Z={msg.z:.1f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = PositionSubscriber()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
