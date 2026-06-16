import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.node import Node

from px4_msgs.msg import VehicleAttitude
import math

from geometry_msgs.msg import Point
import time
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

        self.filtered_area = 0.0
        self.target_z = -14.0

        self.kp_x = 0.0015
        self.kp_z = 0.00015
        self.kp_area = 0.00008

        self.visible_counter = 0
        self.lost_counter = 0


        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_time = time.time()

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_world_x = 0.0
        self.target_world_y = 0.0

        self.prev_target_world_x = 0.0
        self.prev_target_world_y = 0.0

        self.world_vx = 0.0
        self.world_vy = 0.0

        self.desired_area = 2200

        self.vehicle_yaw = 0.0
        self.kp_yaw = 0.002
        self.last_seen_time = time.time()
        self.search_mode = False
        self.last_seen_error_x = 0.0

        self.prediction_time = 0.4
        self.intercept_gain = 0.0

        self.memory_time = 2.0
        self.last_target_world_x = 0.0
        self.last_target_world_y = 0.0



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
            0.05,
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

        self.filtered_area = (
            0.9 * self.filtered_area +
            0.1 * self.area
        )

        if self.area > 100:

            self.visible_counter += 1
            self.lost_counter = 0

            self.last_seen_time = time.time()
            self.last_seen_error_x = self.error_x

            current_time = time.time()

            dt = current_time - self.prev_time

            if dt > 0.01:

                self.target_vx = (
                    self.error_x -
                    self.prev_error_x
                ) / dt

                self.target_vy = (
                    self.error_y -
                    self.prev_error_y
                ) / dt
            


            self.prev_error_x = self.error_x
            self.prev_error_y = self.error_y
            self.prev_time = current_time

            if self.position is not None:

                distance_scale = max(
                    1.0,
                    4000.0 / max(self.area, 1)
                )

                self.target_world_x = (
                    self.position.x +
                    distance_scale *
                    math.cos(self.vehicle_yaw)
                )

                self.target_world_y = (
                    self.position.y +
                    distance_scale *
                    math.sin(self.vehicle_yaw)
                )

                jump = math.sqrt(
                    (self.target_world_x - self.prev_target_world_x)**2 +
                    (self.target_world_y - self.prev_target_world_y)**2
                )

                if jump > 3.0:
                    self.target_world_x = self.prev_target_world_x
                    self.target_world_y = self.prev_target_world_y
                raw_vx = (
                    self.target_world_x -
                    self.prev_target_world_x
                ) / dt

                raw_vy = (
                    self.target_world_y -
                    self.prev_target_world_y
                ) / dt

                self.world_vx = (
                    0.8 * self.world_vx +
                    0.2 * raw_vx
                )

                self.world_vy = (
                    0.8 * self.world_vy +
                    0.2 * raw_vy
                )

                self.prev_target_world_x = self.target_world_x
                self.prev_target_world_y = self.target_world_y

                self.last_target_world_x = self.target_world_x
                self.last_target_world_y = self.target_world_y

        else:

            self.lost_counter += 1


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

        target_visible = (
            
            self.lost_counter < 20
        )

        future_ex = self.error_x
        future_ey = self.error_y
        intercept_offset = 0.0
        

        time_since_seen = (
            time.time() -
            self.last_seen_time
        )

        if target_visible:
            self.search_mode = False

        elif time_since_seen > 3.0:
            self.search_mode = True

        if target_visible:

            predicted_ex = (
                self.error_x +
                self.target_vx *
                self.prediction_time
            )

            predicted_ey = (
                self.error_y +
                self.target_vy *
                self.prediction_time
            )
            
        
            future_ex = (
                self.error_x +
                self.target_vx *
                self.prediction_time
            )

            future_ey = (
                self.error_y +
                self.target_vy *
                self.prediction_time
            )

            future_target_x = (
                self.target_world_x +
                self.world_vx * 1.5
            )

            future_target_y = (
                self.target_world_y +
                self.world_vy * 1.5
            )

            if abs(future_ex) > 20:

                yaw_correction = (
                    self.kp_yaw *
                    future_ex
                )

                yaw_correction = max(
                    -0.4,
                    min(0.4, yaw_correction)
                )

                desired_yaw += yaw_correction

                

            

            # if abs(future_ey) > 20:

            #     z_correction = self.kp_z * future_ey

            #     z_correction = max(
            #         -0.03,
            #         min(0.03, z_correction)
            #     )

            #     self.target_z += z_correction
            

            self.target_z = max(-20.0, min(-2.0, self.target_z))


            if abs(future_ex) < 120:

                area_error = self.desired_area - self.area

                x_correction = self.kp_area * area_error
            

                x_correction = max(
                    -0.2,
                    min(0.2, x_correction)
                )
                if self.counter % 20 == 0:
                    print(
                        f"AREA={self.area:.0f} "
                        f"AREA_ERR={area_error:.0f} "
                        f"X_CORR={x_correction:.3f}"
                    )

                intercept_offset = (
                    future_ex *
                    self.intercept_gain
                )

                forward_cmd = (
                    x_correction +
                    intercept_offset
                )

                forward_cmd = max(
                    -0.3,
                    min(0.3, forward_cmd)
                )

                target_x += (
                    forward_cmd *
                    math.cos(self.vehicle_yaw)
                )

                target_y += (
                    forward_cmd *
                    math.sin(self.vehicle_yaw)
                )

        else:

            if time_since_seen < self.memory_time:

                target_x = self.last_target_world_x
                target_y = self.last_target_world_y

                if self.counter % 20 == 0:
                    print(
                        f"MEMORY "
                        f"TX={target_x:.1f} "
                        f"TY={target_y:.1f} "
                        f"T={time_since_seen:.1f}"
                    )

            else:

                target_x = self.position.x
                target_y = self.position.y

                if self.search_mode:

                    if self.last_seen_error_x > 0:

                        desired_yaw += 0.10

                    else:

                        desired_yaw -= 0.10


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

            if target_visible:
                print(
                    f"VIS={target_visible} "
                    f"AREA={self.area:.0f} "
                    f"SEARCH={self.search_mode}"
                    f"VC={self.visible_counter} "
                    f"LC={self.lost_counter} "
                    f"EX={future_ex:.1f} "
                    f"ICPT={intercept_offset:.2f}"


                )
            else:

                print("Search mode - target not visible")


def main():

    rclpy.init()

    node = VisionFollower()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
