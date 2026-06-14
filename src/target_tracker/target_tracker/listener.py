import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('listener')

        self.subscription = self.create_subscription(
            String,
            'chat',
            self.callback,
            10
        )

    def callback(self, msg):

        self.get_logger().info(
            f"Received: {msg.data}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = Listener()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
