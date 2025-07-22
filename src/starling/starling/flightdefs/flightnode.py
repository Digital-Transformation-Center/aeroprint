#!/usr/bin/env python3
"""
flightnode.py: Framework for PX4 offboard control, including arm, wait, and disarm sequence.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.1.0"
__status__ = "Development"

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from math import pi
from std_msgs.msg import Bool, Int8, Float32
from starling.flightdefs.helix import Helix
from starling.flightdefs.path import Path
import starling.flightdefs.flight_status_codes as flight_status_codes
from starling.flightdefs.tf_manager import TransformManager

from px4_msgs.msg import (
    OffboardControlMode,
    BatteryStatus,
    VehicleCommand,
    VehicleOdometry,
    TrajectorySetpoint,
)

import math # Required for math.radians.


class FlightNode(Node):
    """Node for PX4 offboard control, including arm, wait, and disarm sequence."""

    def __init__(self, name="flight_node") -> None:
        super().__init__(name)

        self.get_logger().info("Flight Node Alive!")

        self.tm = TransformManager() # Transform manager for managing coordinates

        # Configure QoS profile for publishing
        # For control messages, RELIABLE might be preferred, but BEST_EFFORT is used
        # in the reference helical_flight_node.py for PX4 publishers.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # To allow late subscribers to get the last message
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create publisher for OffboardControlMode
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )

        # Create publisher for VehicleCommand
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )

        # Create publisher for TrajectorySetpoint
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )


        self.state_publisher = self.create_publisher(
            Int8, "/fcu/out/status", 10
        )

        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.vehicle_odometry_callback,
            qos_profile,
        )

        self.flight_time_est_publisher = self.create_publisher(
            Float32, "/fcu/out/est_time", 10
        )

        self.time_remaining_publisher = self.create_publisher(
            Float32, "/fcu/out/time_remaining", 10
        )

        # Timer for the main control loop (e.g., every 0.1 seconds = 10Hz)
        # PX4 typically requires OffboardControlMode messages at >2Hz.
        self.control_timer_period = 0.1  # seconds

        # State variables for the arm/disarm sequence
        self.current_system_state = "INIT"  # States: INIT, OFFBOARD_ENGAGING, ARMED, DISARMED_IDLE
        self.offboard_mode_set_time = None
        self.armed_time = None
        self.initial_heartbeat_count = 0
        # Delay start to allow the vehicle to stabilize
        self.init_heartbeat_hold_time = 5.0  # seconds
        self.init_takeoff_hold_time = 10.0

        # Homing variables
        self.home_position = [0.0, 0.0, 0.0]
        self.home_yaw = 0.0  # radians
        self.current_vehicle_position = [0.0, 0.0, 0.0]
        self.current_vehicle_yaw = 0.0
        self.current_vehicle_position_timestamp = None
        self.did_update_home_position = False
        self.home_position_update_time = None
        self.hold_yaw = 0.0  # radians, yaw to hold during takeoff and landing

        self.flight_start_time = None

        self.is_landing = False
        self.figure8_timer = None

        self.rate = 20  # Hz, rate of the helix path

    def init_heartbeat(self, period: float = 0.5):
        self.heartbeat_manager = ServerHeartBeatManager(node = self, acceptable_loss = period)
        # Land the drone if the heartbeat is lost
        self.heartbeat_manager.set_dead_callback(self._land_vehicle())

    def update_path(self, new_path: Path):
        """Update the system path with a new path object"""
        self.path = new_path
        self.start_position = self.path.get_starting_position(invert_axis='z')
        self.end_position = self.path.get_ending_position(invert_axis='z')
        self.steps = self.path.get_num_steps()
        self.hit_flight = False
        self.offboard_arr_counter = 0
        self.publish_time_estimate(self.path.get_duration())
        self.publish_status(flight_status_codes.FLIGHT_PATH_LOADED)

    def publish_time_estimate(self, time_estimate: float) -> None:
        """Publish the estimated flight time."""
        msg = Float32()
        msg.data = time_estimate
        self.flight_time_est_publisher.publish(msg)

    def publish_time_remaining(self, time_remaining: float) -> None:
        """Publish the estimated time remaining for the flight."""
        msg = Float32()
        msg.data = time_remaining
        self.time_remaining_publisher.publish(msg)

    def publish_status(self, status_code: int) -> None:
        """Publish the current flight status code."""
        msg = Int8()
        msg.data = status_code
        self.state_publisher.publish(msg)

    def start_flight(self, msg: Bool) -> None:
        """Callback function to start the flight sequence."""
        self.hit_flight = False
        if msg.data:
            if hasattr(self, "control_timer") and self.control_timer is not None:
                self.control_timer.cancel()
            self.get_logger().info("Start flight command received.")
            self.current_system_state = "INIT"
            if self.current_system_state == "INIT":
                self.initial_heartbeat_count = 0
                self.control_timer = self.create_timer(
                self.control_timer_period, self._control_loop_callback
        ) 
        else:
            self.get_logger().info("Entering landing sequence.")
            self.current_system_state = "LANDING"
            self.is_landing = True
            if hasattr(self, "external_land_callback") and self.external_land_callback:
                self.external_land_callback()
                
    def offboard_move_callback(self):
        if self.offboard_arr_counter < self.steps:
            state = self.path.get_state(self.offboard_arr_counter)
            position = state.get_position(invert_axis='z')
            yaw = state.get_yaw()
            velocity = state.get_velocity()
            acceleration = state.get_acceleration()
            yawspeed = state.get_yaw_rate()
            self.get_logger().info(f"yaw rate : {yawspeed}")
            self._publish_trajectory_setpoint(
                position, yaw, velocity=velocity, acceleration=acceleration, yawspeed=yawspeed
            )   
            self.publish_time_remaining(self.path.get_duration() *  (1-(self.offboard_arr_counter / self.steps)))

        if self.offboard_arr_counter >= self.steps:
            self._publish_trajectory_setpoint(
                [self.end_position.x, self.end_position.y, self.end_position.z], 
                self.hold_yaw, velocity=[0.0, 0.0, 0.0], acceleration=[0.0, 0.0, 0.0], yawspeed=0.0
            )

        # We will hold position so the drone can settle, then land
        if self.offboard_arr_counter == self.steps + 50:
            self.figure8_timer.cancel()
            self.current_system_state = "LANDING"
            self.is_landing = True
            self.get_logger().info("Figure path completed, landing now.")
            if hasattr(self, "external_land_callback") and self.external_land_callback:
                self.external_land_callback()

        self.offboard_arr_counter += 1


    def set_external_arm_callback(self, callback):
        """Set an external callback to engage the offboard control mode."""
        self.external_arm_callback = callback
        self.get_logger().info("External arm callback set.")

    def set_external_takeoff_callback(self, callback):
        """Set an external callback to takeoff the vehicle."""
        self.external_takeoff_callback = callback
        self.get_logger().info("External takeoff callback set.")

    def set_external_engage_callback(self, callback):
        """Set an external callback to engage the offboard control mode."""
        self.external_engage_callback = callback
        self.get_logger().info("External engage callback set.")

    def set_external_land_callback(self, callback):
        """Set an external callback to land the vehicle."""
        self.external_land_callback = callback
        self.get_logger().info("External land callback set.")

    def set_external_disarm_callback(self, callback):
        """Set an external callback to disarm the vehicle."""
        self.external_disarm_callback = callback
        self.get_logger().info("External disarm callback set.")

    def _control_loop_callback(self) -> None:
        """Main control loop executed by the timer."""
        self._publish_offboard_control_heartbeat()

        if self.current_system_state == "INIT":
            self.initial_heartbeat_count += 1
            if self.initial_heartbeat_count >= int(self.init_heartbeat_hold_time / self.control_timer_period):  # Wait for ~1 second of heartbeats
                self._update_home_position()
                self._engage_offboard_mode()
                self.offboard_mode_set_time = self.get_clock().now()
                self.current_system_state = "OFFBOARD_ENGAGING"
                self.get_logger().info("Attempting to engage Offboard mode.")
                if hasattr(self, "external_arm_callback") and self.external_arm_callback:
                    self.external_arm_callback()

        elif self.current_system_state == "OFFBOARD_ENGAGING":
            # Wait a short period for mode switch to take effect (e.g., 0.5s)
            if self.offboard_mode_set_time and \
               self.get_clock().now() - self.offboard_mode_set_time >= Duration(seconds=0.5):
                self._arm_vehicle()
                self.armed_time = self.get_clock().now()
                self.current_system_state = "ARMED"
                self.get_logger().info("Vehicle arm command sent.")
                if hasattr(self, "external_takeoff_callback") and self.external_takeoff_callback:
                    self.external_takeoff_callback()

        elif self.current_system_state == "ARMED":
            if self.armed_time and \
               self.get_clock().now() - self.armed_time >= Duration(seconds=self.init_takeoff_hold_time):
                self.flight_start_time = self.get_clock().now()
                self.current_system_state = "FLIGHT_ENGAGED"
                self.get_logger().info("Takeoff complete. Starting flight sequence.")
                if hasattr(self, "external_engage_callback") and self.external_engage_callback:
                    self.external_engage_callback()
            else:
                try:
                    self._publish_trajectory_setpoint([self.start_position.x, self.start_position.y, self.start_position.z], self.hold_yaw)
                    self.publish_status(flight_status_codes.FLIGHT_ARMED)
                except AssertionError as e:
                    self.get_logger().error(f"Error publishing trajectory setpoint: {e}")
                    self.current_system_state = "DISARMED_IDLE"
                    self.get_logger().info("Disarming due to error in trajectory setpoint.")
                    self._disarm_vehicle()
                    self.get_logger().info(f"Attempted position: {self.start_position.x}, {self.start_position.y}, {self.start_position.z}")
                    return

        elif self.current_system_state == "FLIGHT_ENGAGED":
            # Only create the timer if it doesn't already exist
            if not self.hit_flight:
                self.publish_status(flight_status_codes.FLIGHT_ENGAGED)
                self.figure8_timer = self.create_timer(
                    1 / self.rate, self.offboard_move_callback
                )
                self.hit_flight = True
                    

        elif self.current_system_state == "LANDING":
            self._land_vehicle()
            if self.figure8_timer is not None:
                self.figure8_timer.cancel()

            if self.hit_flight or self.is_landing:
                self.land_counter = 0
                self.is_landing = False
                self.hit_flight = False
                self.publish_status(flight_status_codes.FLIGHT_LANDING)
            
            if self.land_counter > 50:  # Wait for 5 seconds at 10Hz
                self.current_system_state = "DISARMED_IDLE"
                if hasattr(self, "external_disarm_callback") and self.external_disarm_callback:
                    self.external_disarm_callback()

            self.land_counter += 1
            # msg = Int8()
            # msg.data = 0  # Indicating disarmed state
            # self.state_publisher.publish(msg)
            # self.get_logger().info("Vehicle land command sent.")
            # # self.destroy_node()


        elif self.current_system_state == "DISARMED_IDLE":
            self.publish_status(flight_status_codes.FLIGHT_DISARMED)
            if hasattr(self, "control_timer") and self.control_timer is not None:
                self.control_timer.cancel()
                self.hit_flight = False
            # The control_timer should have been cancelled.
            # If not, something is wrong or it was restarted.

    def _publish_offboard_control_heartbeat(self) -> None:
        """Publish the offboard control mode signal (heartbeat)."""
        msg = OffboardControlMode()
        msg.position = True  # We intend to control position
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.offboard_control_mode_publisher.publish(msg)

    def _publish_trajectory_setpoint(self, position: list, yaw: float, velocity: list = None, acceleration: list = None, yawspeed: float = None) -> None:
        """Publish the trajectory setpoint, adjusting position, acceleration, and yaw based on home position and yaw."""
        msg = TrajectorySetpoint()

        # # Helper to rotate a vector by a given yaw (radians)
        # def rotate_vector(vec, yaw):
        #     if vec is None:
        #         return None
        #     if len(vec) != 3:
        #         raise ValueError("Vector must be length 3")
        #     cos_yaw = math.cos(yaw)
        #     sin_yaw = math.sin(yaw)
        #     x, y, z = vec
        #     x_r = x * cos_yaw - y * sin_yaw
        #     y_r = x * sin_yaw + y * cos_yaw
        #     return [float(x_r), float(y_r), float(z)]

        # # Adjust position: rotate by home_yaw, then translate by home_position
        # rotated_position = rotate_vector([float(p) for p in position], self.home_yaw)
        # msg.position = [rotated_position[i] + float(self.home_position[i]) for i in range(3)]

        # # Adjust acceleration: rotate by home_yaw
        # if acceleration is not None:
        #     msg.acceleration = rotate_vector(acceleration, self.home_yaw)

        # # Adjust velocity: rotate by home_yaw
        # if velocity is not None:
        #     msg.velocity = rotate_vector(velocity, self.home_yaw)

        # # Adjust yaw: add home_yaw
        # msg.yaw = float(yaw) + self.home_yaw

        # Adjust yawspeed: no change needed (already in body frame)
        if yawspeed is not None:
            msg.yawspeed = float(yawspeed)
        pos = self.tm.get_position_wrt_world(position)
        if pos is None:
            raise AssertionError("Failed to transform position to world frame.")
        msg.position = pos
        if velocity is not None:
            msg.velocity = self.tm.get_vector_wrt_world(velocity)
        if acceleration is not None:
            msg.acceleration = self.tm.get_vector_wrt_world(acceleration)
        

        yaw_uv = self.tm.get_vector_wrt_world([math.cos(yaw), math.sin(yaw), 0.0])
        msg.yaw = math.atan2(yaw_uv[1], yaw_uv[0])
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def battery_status_callback(self, msg: BatteryStatus) -> None:
        """Callback function for battery_status topic subscriber."""
        # The 'remaining' field is a float from 0.0 (empty) to 1.0 (full)
        # NAN if unknown.
        self.get_logger().info(f"Battery Remaining: {msg.remaining:.1f}%")

    def vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        """Callback function for vehicle_odometry topic subscriber."""
        self.current_vehicle_position = msg.position
        w, x, y, z = msg.q
        self.current_vehicle_yaw = self.get_yaw_from_quaternion(w, x, y, z)
        self.current_vehicle_position_timestamp = self.get_clock().now()
        self.did_update_vehicle_position = True
        
    def _publish_vehicle_command(self, command: int, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0) # For VEHICLE_CMD_DO_SET_MODE: MAV_MODE
        msg.param6 = params.get("param6", 0.0) # For VEHICLE_CMD_DO_SET_MODE: Sub-mode
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1  # System ID
        msg.target_component = 1  # Component ID
        msg.source_system = 1  # System ID of the GCS/companion computer
        msg.source_component = 1  # Component ID of the GCS/companion computer
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().debug(f"VehicleCommand {command} sent.")

    def _engage_offboard_mode(self) -> None:
        """Switch to offboard mode."""
        # VEHICLE_CMD_DO_SET_MODE: param1 = base_mode, param2 = custom_mode
        # For Offboard mode, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (usually 1) in base_mode
        # and PX4_CUSTOM_MAIN_MODE_OFFBOARD (6) in custom_mode (param2 for MAVLink, param6 for uORB Command)
        # In PX4 v1.14+ VEHICLE_CMD_DO_SET_MODE param1 is MAV_MODE, param2 is sub_mode
        # MAV_MODE_OFFBOARD = 6 (for PX4 custom modes this is usually set in param2 or param6)
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Engage Offboard mode command sent.")

    def _arm_vehicle(self) -> None:
        """Send an arm command to the vehicle."""
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0) # 1.0 to arm
        self.get_logger().info("Arm command sent.")

    def _disarm_vehicle(self) -> None:
        """Send a disarm command to the vehicle."""
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0) # 0.0 to disarm
        self.get_logger().info("Disarm command sent.")

    def _land_vehicle(self) -> None:
        """Send a land command to the vehicle."""
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param1=0.0, param2=0.0, param3=0.0, param4=0.0)
        self.get_logger().info("Land command sent.")

    def _update_home_position(self) -> None:
        if self.current_vehicle_position_timestamp is None:
            self.get_logger().info("Current vehicle position not available, cannot update home position.")
            rclpy.spin_once(self, timeout_sec=0.1)
            self._update_home_position()  # Retry after a short delay
            return
        elif self.get_clock().now() - self.current_vehicle_position_timestamp < Duration(seconds=0.1):
            self.tm.set_home(
                self.current_vehicle_position, self.current_vehicle_yaw)
            self.tm.wait_for_home_transform()
            self.get_logger().info("Home position updated successfully.")
            # self.home_position = self.current_vehicle_position.copy()
            # self.home_yaw = self.current_vehicle_yaw
            # self.get_logger().info(f"Home position updated: {self.home_position}")
            # self.get_logger().info(f"Home yaw updated: {self.home_yaw} radians")
            return
        else:
            self.get_logger().warn("Current vehicle position is too old, not updating home position.")
            self._update_home_position()
        
    def destroy_node(self):
        self.get_logger().info("Shutting down Heartbeat Node.")
        super().destroy_node()

    def get_yaw_from_quaternion(self, w, x, y, z):
        # Yaw (Z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
# A heart beat manager for the server
class ServerHeartBeatManager():
    def __init__(self, node: Node, acceptable_loss: float) -> None:
        self.node = node
        self.acceptable_loss = acceptable_loss  # seconds
        self.heartbeat_publisher = self.node.create_publisher(
            Int8, '/fcu/out/heartbeat', 10
        )
        self.heartbeat_subscriber = self.node.create_subscription(
            Int8, '/fcu/in/heartbeat', self.heartbeat_callback, 10
        )
        self.last_heartbeat_sent = 0
        self.last_heartbeat_received = self.node.get_clock().now().nanoseconds / 1e9
        self.heartbeat_value = 0
        self.dead_called = False

        # Publish heartbeat at 2x the acceptable_loss frequency
        self.heartbeat_period = acceptable_loss / 2.0
        self.heartbeat_timer = self.node.create_timer(
            self.heartbeat_period, self.send_heartbeat
        )
        # Timer to check for missed heartbeats (check at same rate as heartbeat)
        self.check_timer = self.node.create_timer(
            self.heartbeat_period, self.check_heartbeat
        )

    def send_heartbeat(self):
        msg = Int8()
        self.heartbeat_value = (self.heartbeat_value + 1) % 128  # cycle value for robustness
        msg.data = self.heartbeat_value
        self.heartbeat_publisher.publish(msg)
        self.last_heartbeat_sent = self.node.get_clock().now().nanoseconds / 1e9
        # Optionally, reset dead_called if a new heartbeat is sent
        self.dead_called = False

    def heartbeat_callback(self, msg):
        # Only accept if matches last sent value
        if msg.data == self.heartbeat_value:
            self.last_heartbeat_received = self.node.get_clock().now().nanoseconds / 1e9
            self.dead_called = False
        else:
            self.node.get_logger().error("Heartbeat out of sync.")

    def check_heartbeat(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        if (now - self.last_heartbeat_received) > self.acceptable_loss:
            if not self.dead_called:
                self.dead()
                self.dead_called = True

    def set_dead_callback(self, callback):
        self.dead_callback = callback

    def dead(self):
        self.node.get_logger().warn("Heartbeat lost! Calling dead() handler.")
        if hasattr(self, "dead_callback"):
            self.dead_callback()
        pass


def main(args=None) -> None:
    """
    Main function to initialize and run the FlightNode
    """
    rclpy.init(args=args)
    flight_node = FlightNode()
    try:
        rclpy.spin(flight_node)
    except KeyboardInterrupt:
        flight_node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        flight_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"An error occurred in main: {e}")