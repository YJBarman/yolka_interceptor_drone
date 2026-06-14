import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class TargetController(Node):

    def __init__(self):
        super().__init__('target_controller')

        self.subscription = self.create_subscription(
            Point,
            '/tracking/error',
            self.error_callback,
            10
        )

    def error_callback(self, msg):

        error_x = msg.x

        kp = 0.003

        yaw_rate = -kp * error_x

        self.get_logger().info(
            f"EX={error_x:.1f}  yaw_rate={yaw_rate:.3f}"
        )



def main(args=None):

    rclpy.init(args=args)

    node = TargetController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()