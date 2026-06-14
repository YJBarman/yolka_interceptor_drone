import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

import subprocess
import re


class VisionFollower(Node):

    def __init__(self):
        super().__init__('vision_follower')

        self.subscription = self.create_subscription(
            Point,
            '/tracking/error',
            self.error_callback,
            10
        )

    def get_drone_pose(self):

        result = subprocess.run(
            [
                "gz",
                "topic",
                "-e",
                "-n",
                "1",
                "-t",
                "/world/default/dynamic_pose/info"
            ],
            capture_output=True,
            text=True
        )

        text = result.stdout

        pattern = (
            r'name: "x500_mono_cam_0".*?'
            r'x: ([^ \n]+).*?'
            r'y: ([^ \n]+).*?'
            r'z: ([^ \n]+)'
        )

        m = re.search(pattern, text, re.S)

        if m:
            return (
                float(m.group(1)),
                float(m.group(2)),
                float(m.group(3))
            )

        return None

    def move_drone(self, x, y, z):

        req = f'''
name: "x500_mono_cam_0"

position {{
  x: {x}
  y: {y}
  z: {z}
}}

orientation {{
  w: 1
}}
'''

        subprocess.run(
            [
                "gz",
                "service",
                "-s",
                "/world/default/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--req",
                req
            ],
            capture_output=True
        )

    def error_callback(self, msg):

        error_x = msg.x
        error_y = msg.y
        area = msg.z

        target_area = 5000

        area_error = target_area - area

        pose = self.get_drone_pose()

        if pose is None:
            return

        x, y, z = pose

        # Horizontal tracking gain
        kp_y = 0.001

        # Distance tracking gain
        kp_x = 0.00001

        kp_z = 0.001

        # Deadbands
        if abs(error_x) < 5:
            error_x = 0

        if abs(area_error) < 500:
            area_error = 0
        if abs(error_y) < 5:
            error_y = 0

        # Horizontal control
        y = y - kp_y * error_x

        # Distance control
        x = x + kp_x * area_error

        # Vertical control
        # Vertical control
        z = z - kp_z * error_y

        self.move_drone(x, y, z)

        self.get_logger().info(
            f"EX={error_x:.1f} "
            f"AREA={area:.0f} "
            f"AERR={area_error:.0f} "
            f"X={x:.2f} "
            f"Y={y:.2f} "
            f"Z={z:.2f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = VisionFollower()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

