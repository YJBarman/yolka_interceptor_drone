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

        self.t += 0.02

        radius = 4.0
        speed = 2.0

        x = 5.0 + radius * math.cos(speed * self.t)
        y = 5.0 + radius * math.sin(speed * self.t)
        z = 12.0
        
       

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