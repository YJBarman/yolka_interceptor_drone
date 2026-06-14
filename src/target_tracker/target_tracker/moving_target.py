import rclpy
from rclpy.node import Node

import subprocess
import math


class MovingTarget(Node):

    def __init__(self):
        super().__init__('moving_target')

        self.t = 0.0

        self.timer = self.create_timer(
            0.01,
            self.move_target
        )

    def move_target(self):

        self.t += 0.03

        radius = 3.0

        x = radius * math.cos(self.t)
        y = radius * math.sin(self.t)

        req = f'''
name: "x500_0"

position {{
  x: {x}
  y: {y}
  z: -0.013
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
                "/world/kthspacelab/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--req",
                req
            ],
            stdout=subprocess.DEVNULL
        )


def main(args=None):

    rclpy.init(args=args)

    node = MovingTarget()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
