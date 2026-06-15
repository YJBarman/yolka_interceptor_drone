import rclpy
from rclpy.node import Node

import math
import subprocess
import time

class MovingTarget(Node):

    def __init__(self):

        super().__init__('moving_target')

        self.t = 0.0

        self.timer = self.create_timer(
            0.02,
            self.timer_callback
        )

    def timer_callback(self):

        self.t += 0.1

        x = 5.0 + 3.0 * math.cos(0.4 * self.t)
        y = 3.0 * math.sin(0.4 * self.t)
        z = 2.0
        print(f"x={x:.2f} y={y:.2f} z={z:.2f}")

        req = f'''
name: "target_ball"
position {{
  x: {x}
  y: {y}
  z: {z}
}}
orientation {{
  w: 1
}}
'''
        start = time.time()
        subprocess.run([
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
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL)
        elapsed = time.time() - start

        print(f"service time = {elapsed:.3f}")


def main():

    rclpy.init()

    node = MovingTarget()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()