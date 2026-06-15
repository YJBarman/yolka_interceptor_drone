import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.node import Node

from px4_msgs.msg import VehicleAttitude
import math

from geometry_msgs.msg import Point

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition
)

from math import nan


class VisionFollower(Node):

    def __init__(self):

        super().__init__('vision_px4_follower')

        self.position = None
    
        self.error_x = 0.0
        self.error_y = 0.0
        self.area = 0.0
        self.target_z = -3.0

        self.kp_x = 0.0015
        self.kp_z = 0.00015
        self.kp_area = 0.00008

        self.desired_area = 2200

        self.vehicle_yaw = 0.0
        self.kp_yaw = 0.002

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(
            Point,
            '/tracking/error',
            self.tracking_callback,
            10
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback,
            px4_qos
        )

        self.create_subscription(
                VehicleAttitude,
                '/fmu/out/vehicle_attitude',
                self.attitude_callback,
                px4_qos
            )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.counter = 0

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )


    def attitude_callback(self, msg):

        q = msg.q

        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]

        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)

        self.vehicle_yaw = math.atan2(
            siny,
            cosy
        )

    def tracking_callback(self, msg):

        self.error_x = msg.x
        self.error_y = msg.y
        self.area = msg.z

    def local_position_callback(self, msg):

        self.position = msg

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.command = command

        msg.param1 = param1
        msg.param2 = param2

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.cmd_pub.publish(msg)

    def timer_callback(self):

        if self.position is None:
            return
        
        
        offboard = OffboardControlMode()

        offboard.timestamp = self.get_clock().now().nanoseconds // 1000

        offboard.position = True

        self.offboard_pub.publish(offboard)

        target_x = self.position.x
        target_y = self.position.y

        desired_yaw = self.vehicle_yaw

        target_visible = self.area > 100

        if target_visible:
        
            if abs(self.error_x) > 20:

                yaw_correction = (
                    self.kp_yaw *
                    self.error_x
                )

                yaw_correction = max(
                    -0.4,
                    min(0.4, yaw_correction)
                )

                desired_yaw += yaw_correction

                print(
                    f"EX={self.error_x:.0f} "
                    f"YAW_CORR={yaw_correction:.3f}"
                )

            if abs(self.error_y) > 20:

                z_correction = self.kp_z * self.error_y

                z_correction = max(
                    -0.03,
                    min(0.03, z_correction)
                )

                self.target_z += z_correction

            self.target_z = max(-8.0, min(-0.3, self.target_z))


            if abs(self.error_x) < 100:

                area_error = self.desired_area - self.area

                x_correction = self.kp_area * area_error
                if self.counter % 20 == 0:
                    print(
                        f"AREA={self.area:.0f} "
                        f"AREA_ERR={area_error:.0f} "
                        f"X_CORR={x_correction:.3f}"
                    )

                x_correction = max(
                    -0.2,
                    min(0.2, x_correction)
                )

                target_x += x_correction * math.cos(self.vehicle_yaw)
                target_y += x_correction * math.sin(self.vehicle_yaw)


        sp = TrajectorySetpoint()

        sp.timestamp = self.get_clock().now().nanoseconds // 1000

        sp.position = [
            target_x,
            target_y,
            self.target_z
        ]

        sp.velocity = [nan, nan, nan]

        sp.yaw = desired_yaw

        self.traj_pub.publish(sp)

        self.counter += 1

        if self.counter == 20:

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

        if self.counter == 25:

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

        if self.counter % 20 == 0:

            print(
                f"VISIBLE={self.area > 100} "
                f"EX={self.error_x:.0f} "
                f"EY={self.error_y:.0f} "
                f"TZ_CMD={self.target_z:.2f} "
                f"REAL_Z={self.position.z:.2f} "
                f"AREA={self.area:.0f} "
                f"TX={target_x:.2f} "
                f"TY={target_y:.2f} "
                f"TZ={self.target_z:.2f}"
            )


def main():

    rclpy.init()

    node = VisionFollower()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
