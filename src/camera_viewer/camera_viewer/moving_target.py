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

        self.t += 0.5

        x = 5.0 + 3.0 * math.cos(0.4 * self.t)
        side = 3.0

        phase = int(self.t) % 40

        if phase < 10:
            x = 5.0
            y = phase * side / 10

        elif phase < 20:
            x = 5.0 - (phase-10) * side / 10
            y = side

        elif phase < 30:
            x = 2.0
            y = side - (phase-20) * side / 10

        else:
            x = 2.0 + (phase-30) * side / 10
            y = 0.0
        z = 7.0
        if int(self.t) % 10 == 0:
            print(f"TARGET x={x:.1f} y={y:.1f}")

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

        


def main():

    rclpy.init()

    node = MovingTarget()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()