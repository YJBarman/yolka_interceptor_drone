import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
import math


class Tracker(Node):

    def __init__(self):
        super().__init__('tracker')

        self.subscription = self.create_subscription(
            Point,
            'target_position',
            self.callback,
            10
        )

        self.chaser_x = 0.0
        self.chaser_y = 0.0
        self.chaser_z = 10.0

    def callback(self, msg):

        dx = msg.x - self.chaser_x
        dy = msg.y - self.chaser_y
        dz = msg.z - self.chaser_z

        distance = math.sqrt(
            dx**2 +
            dy**2 +
            dz**2
        )

        self.get_logger().info(
            f"Distance to target = {distance:.2f} m"
        )


def main(args=None):

    rclpy.init(args=args)

    node = Tracker()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
