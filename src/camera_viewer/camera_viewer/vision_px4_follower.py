import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.node import Node
import math
import time
import numpy as np
from geometry_msgs.msg import Point
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude
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
        self.target_z = -2.5

        # Control Gains
        self.kp_yaw = 0.0025
        self.kp_track_dist = 0.25  # Proportional gain for world-space tracking distance error

        self.visible_counter = 0
        self.lost_counter = 0
        self.prev_time = time.time()

        # Target world states (updated by Kalman Filter)
        self.target_world_x = 0.0
        self.target_world_y = 0.0
        self.world_vx = 0.0
        self.world_vy = 0.0

        self.prev_target_world_x = 0.0
        self.prev_target_world_y = 0.0

        self.desired_area = 2200
        self.distance_calib_constant = 140.0 

        # Derived ideal distance profile in meters (e.g., 140 / sqrt(2200) = ~3.0 meters)
        self.desired_distance = self.distance_calib_constant / math.sqrt(self.desired_area)

        # Kalman Filter Setup
        self.kf_initialized = False
        self.kf_x = np.zeros((4, 1))
        self.kf_P = np.eye(4) * 10.0
        self.kf_H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])
        self.kf_R = np.eye(2) * 0.15  # Tuned down measurement variance slightly based on clean logs
        self.kf_Q = np.eye(4) * 0.08  # Process noise allows filter to quickly adjust to target turns

        self.vehicle_yaw = 0.0
        self.last_seen_time = time.time() - 10.0
        self.search_mode = True
        self.last_seen_error_x = 0.0

        # --- STEP 3 PARAMETERS ---
        self.prediction_time = 0.5  # Look-ahead time horizon (seconds)
        self.memory_time = 2.0
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(Point, '/tracking/error', self.tracking_callback, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_callback, px4_qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, px4_qos)

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.counter = 0
        self.timer = self.create_timer(0.05, self.timer_callback)

    def attitude_callback(self, msg):
        q = msg.q
        w, x, y, z = q[0], q[1], q[2], q[3]
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        self.vehicle_yaw = math.atan2(siny, cosy)

    def tracking_callback(self, msg):
        self.error_x = msg.x
        self.error_y = msg.y
        self.area = msg.z

        self.filtered_area = 0.9 * self.filtered_area + 0.1 * self.area

        if self.area > 100:
            self.visible_counter += 1
            self.lost_counter = 0
            current_time = time.time()
            dt = current_time - self.prev_time
            self.last_seen_time = current_time
            self.last_seen_error_x = self.error_x

            if dt < 0.001:
                dt = 0.05 

            self.prev_time = current_time

            if self.position is not None:
                # Square root distance calculation
                distance_scale = max(1.0, self.distance_calib_constant / math.sqrt(max(self.area, 1.0)))

                raw_target_world_x = self.position.x + distance_scale * math.cos(self.vehicle_yaw)
                raw_target_world_y = self.position.y + distance_scale * math.sin(self.vehicle_yaw)

                jump = math.sqrt((raw_target_world_x - self.prev_target_world_x)**2 + (raw_target_world_y - self.prev_target_world_y)**2)
                if jump > 4.0 and self.kf_initialized:
                    raw_target_world_x = self.prev_target_world_x
                    raw_target_world_y = self.prev_target_world_y

                self.prev_target_world_x = raw_target_world_x
                self.prev_target_world_y = raw_target_world_y

                # Kalman State Updates
                if not self.kf_initialized:
                    self.kf_x = np.array([[raw_target_world_x], [raw_target_world_y], [0.0], [0.0]])
                    self.kf_P = np.eye(4) * 1.0
                    self.kf_initialized = True
                else:
                    F = np.array([
                        [1.0, 0.0,   dt, 0.0],
                        [0.0, 1.0,  0.0,  dt],
                        [0.0, 0.0,  1.0, 0.0],
                        [0.0, 0.0,  0.0, 1.0]
                    ])

                    self.kf_x = F @ self.kf_x
                    self.kf_P = F @ self.kf_P @ F.T + self.kf_Q

                    z = np.array([[raw_target_world_x], [raw_target_world_y]])
                    y_residual = z - (self.kf_H @ self.kf_x)
                    S = self.kf_H @ self.kf_P @ self.kf_H.T + self.kf_R
                    K = self.kf_P @ self.kf_H.T @ np.linalg.inv(S)

                    self.kf_x = self.kf_x + K @ y_residual
                    self.kf_P = (np.eye(4) - K @ self.kf_H) @ self.kf_P

                self.target_world_x = self.kf_x[0, 0]
                self.target_world_y = self.kf_x[1, 0]
                self.world_vx = self.kf_x[2, 0]
                self.world_vy = self.kf_x[3, 0]
        else:
            self.lost_counter += 1

    def local_position_callback(self, msg):
        self.position = msg

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
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
        
        time_since_seen = time.time() - self.last_seen_time
        actual_target_visible = (self.area > 100) and (time_since_seen < 0.25)

        if actual_target_visible:
            self.search_mode = False
            
            # 1. Compute Target's Future Spatial Intercept Point
            pred_target_x = self.target_world_x + (self.world_vx * self.prediction_time)
            pred_target_y = self.target_world_y + (self.world_vy * self.prediction_time)

            # 2. Geometric Distance Tracking vector calculations
            dx = pred_target_x - self.position.x
            dy = pred_target_y - self.position.y
            current_distance_to_pred = math.sqrt(dx**2 + dy**2)

            if current_distance_to_pred > 0.1:
                # Calculate vector direction angle towards the future interception coordinate
                interception_heading = math.atan2(dy, dx)
                
                # Close the gap between current state and tracking footprint
                distance_error = current_distance_to_pred - self.desired_distance
                step_magnitude = self.kp_track_dist * distance_error
                step_magnitude = max(-0.4, min(0.4, step_magnitude)) # Limit target generation velocity step

                # Project coordinate step positions dynamically
                target_x = self.position.x + step_magnitude * math.cos(interception_heading)
                target_y = self.position.y + step_magnitude * math.sin(interception_heading)

            # 3. Dynamic Yaw Camera Centering (Align to target tracking coordinate framework)
            if abs(self.error_x) > 15:
                yaw_correction = self.kp_yaw * self.error_x
                yaw_correction = max(-0.3, min(0.3, yaw_correction))
                desired_yaw += yaw_correction

            self.target_z = max(-20.0, min(-2.0, self.target_z))

            if self.counter % 20 == 0:
                print(f"INTERCEPT ACTIVE: Dist_To_Pred={current_distance_to_pred:.1f}m | "
                      f"Target_Pred=[{pred_target_x:.1f}, {pred_target_y:.1f}] | "
                      f"Vel_Vector=[{self.world_vx:.2f}, {self.world_vy:.2f}]")

        elif time_since_seen < self.memory_time:
            # --- PROPAGATED MEMORY DEAD RECKONING ---
            self.search_mode = False
            
            # Predict location tracking through drop frame sequences smoothly
            target_x = self.target_world_x + (self.world_vx * time_since_seen)
            target_y = self.target_world_y + (self.world_vy * time_since_seen)
            
            if self.counter % 20 == 0:
                print(f"MEMORY INTERCEPT: Moving to Predicted Route -> TX={target_x:.1f} TY={target_y:.1f}")

        else:
            # --- SEARCH PROFILE SCANNING ---
            self.search_mode = True
            self.kf_initialized = False 
            target_x = self.position.x
            target_y = self.position.y

            if time_since_seen > 3.0:
                if self.last_seen_error_x > 0:
                    desired_yaw += 0.08
                else:
                    desired_yaw -= 0.08

            if self.counter % 20 == 0:
                print("Search mode - target scanning")

        sp = TrajectorySetpoint()
        sp.timestamp = self.get_clock().now().nanoseconds // 1000
        sp.position = [target_x, target_y, self.target_z]
        sp.velocity = [nan, nan, nan]
        sp.yaw = desired_yaw
        self.traj_pub.publish(sp)

        self.counter += 1
        if self.counter == 20:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if self.counter == 25:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

def main():
    rclpy.init()
    node = VisionFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()