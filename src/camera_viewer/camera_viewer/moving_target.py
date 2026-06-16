import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist

class MovingTarget(Node):

    def __init__(self):
        super().__init__('moving_target')

        self.t = 0.0

        # Create a native ROS2 topic publisher
        self.vel_pub = self.create_publisher(
            Twist,
            '/target_ball/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.02,
            self.timer_callback
        )

    def timer_callback(self):
        self.t += 0.02

        radius = 4.0
        speed = 0.5
        z = 2.0  # Float the ball slightly below the drone's plane

        # Derivative of circular motion equations:
        # x = 5 + r*cos(w*t) -> vx = -r * w * sin(w*t)
        # y = 5 + r*sin(w*t) -> vy =  r * w * cos(w*t)
        vx = -radius * speed * math.sin(speed * self.t)
        vy = radius * speed * math.cos(speed * self.t)

        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0  # Smoothly hold altitude layer

        self.vel_pub.publish(msg)

def main():
    rclpy.init()
    node = MovingTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()