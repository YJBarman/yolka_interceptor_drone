import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


class PosePrinter(Node):

    def __init__(self):
        super().__init__('pose_printer')

        self.subscription = self.create_subscription(
            Imu,
            '/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):

        self.get_logger().info(
            f"Gyro Z = {msg.angular_velocity.z:.6f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = PosePrinter()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

