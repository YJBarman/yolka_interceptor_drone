import rclpy
from rclpy.node import Node

import subprocess
import re
import math
import time

class Follower(Node):

    def __init__(self):
        super().__init__('follower')
        self.prev_target_x = None
        self.prev_target_y = None
        self.prev_time = None

        self.timer = self.create_timer(
            0.5,
            self.follow_callback
        )

    def get_position(self, name):

        result = subprocess.run(
            [
                "gz",
                "topic",
                "-e",
                "-n",
                "1",
                "-t",
                "/world/kthspacelab/dynamic_pose/info"
            ],
            capture_output=True,
            text=True
        )

        text = result.stdout

        pattern = (
            rf'name: "{name}".*?position \{{.*?x: ([^ \n]+).*?y: ([^ \n]+).*?z: ([^ \n]+)'
        )

        m = re.search(pattern, text, re.S)

        if m:
            return (
                float(m.group(1)),
                float(m.group(2)),
                float(m.group(3))
            )

        return None

    def move_chaser(self, x, y, z):

        req = f'''
name: "x500_1"

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
                "/world/kthspacelab/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--req",
                req
            ]
        )

    def follow_callback(self):

        target = self.get_position("x500_0")
        chaser = self.get_position("x500_1")

        if target is None or chaser is None:
            return

        tx, ty, tz = target
        cx, cy, cz = chaser

        current_time = time.time()

        if self.prev_target_x is None:

           self.prev_target_x = tx
           self.prev_target_y = ty
           self.prev_time = current_time

           return

        dt = current_time - self.prev_time

        if dt <= 0:
           return

        vx = (tx - self.prev_target_x) / dt
        vy = (ty - self.prev_target_y) / dt

        lookahead = 1.5

        future_x = tx + vx * lookahead
        future_y = ty + vy * lookahead

        self.prev_target_x = tx
        self.prev_target_y = ty
        self.prev_time = current_time

        dx = future_x - cx
        dy = future_y - cy

        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.1:
            return

        step = 0.5

        cx += step * dx / distance
        cy += step * dy / distance

        self.move_chaser(
            cx,
            cy,
            cz
        )

        self.get_logger().info(
    f"Distance={distance:.2f} "
    f"Vx={vx:.2f} "
    f"Vy={vy:.2f}"
)


def main(args=None):

    rclpy.init(args=args)

    node = Follower()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
