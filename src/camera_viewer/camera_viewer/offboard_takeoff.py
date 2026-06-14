import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition
)

from math import nan


class OffboardTakeoff(Node):

    def __init__(self):
        super().__init__('offboard_takeoff')

        self.position = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_pos_callback,
            qos_profile
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

    def local_pos_callback(self, msg):
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
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False

        self.offboard_pub.publish(offboard)

        sp = TrajectorySetpoint()
        sp.timestamp = self.get_clock().now().nanoseconds // 1000

        sp.position = [0.0, 0.0, -3.0]
        sp.velocity = [nan, nan, nan]
        sp.acceleration = [nan, nan, nan]
        sp.yaw = 0.0

        self.traj_pub.publish(sp)

        self.counter += 1

        if self.counter == 20:

            print("Switching to OFFBOARD")

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

        if self.counter == 25:

            print("ARM")

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

        if self.counter % 20 == 0:

            print(
                f"x={self.position.x:.2f} "
                f"y={self.position.y:.2f} "
                f"z={self.position.z:.2f}"
            )


def main():

    rclpy.init()

    node = OffboardTakeoff()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
