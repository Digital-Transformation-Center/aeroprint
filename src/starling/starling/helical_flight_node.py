#!/usr/bin/env python3
"""
new_flight.py: Basic ROS 2 node to send a heartbeat signal for PX4 offboard control.
UDRI DTC AEROPRINT
"""
__author__ = "Gemini Code Assist"
__email__ = "your.email@example.com"
__version__ = "0.1.0"
__status__ = "Development"

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from math import pi
from std_msgs.msg import Bool, Float32MultiArray, Int8
from starling.flightdefs.helix import Helix

from px4_msgs.msg import (
    OffboardControlMode,
    BatteryStatus,
    VehicleCommand,
    VehicleOdometry,
    TrajectorySetpoint,
)

import math # Required for math.radians if used for yaw, though 0.0 is fine.


class HeartbeatNode(Node):
    """Node for PX4 offboard control, including arm, wait, and disarm sequence."""

    def __init__(self) -> None:
        super().__init__("heartbeat_node")

        self.get_logger().info("Heartbeat Node Alive!")

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
            Int8, "/state", 10
        )

        self.start_flight_subscriber = self.create_subscription(
            Bool, 
            "/start_flight",
            self.start_flight,
            10
        )

        self.param_subscriber = self.create_subscription(
            Float32MultiArray, 'helix_params', self.helix_params_callback, 10
        )

        # Create subscriber for BatteryStatus
        # self.battery_status_subscriber = self.create_subscription(
        #     BatteryStatus,
        #     "/fmu/out/battery_status",
        #     self.battery_status_callback,
        #     qos_profile,
        # )

        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.vehicle_odometry_callback,
            qos_profile,
        )

        # Timer for the main control loop (e.g., every 0.1 seconds = 10Hz)
        # PX4 typically requires OffboardControlMode messages at >2Hz.
        self.control_timer_period = 0.1  # seconds
        

        # State variables for the arm/disarm sequence
        self.current_system_state = "INIT"  # States: INIT, OFFBOARD_ENGAGING, ARMED, DISARMED_IDLE
        self.offboard_mode_set_time = None
        self.armed_time = None
        self.initial_heartbeat_count = 0
        self.init_heartbeat_hold_time = 5.0  # seconds
        self.init_takeoff_hold_time = 15.0

        # Target for TrajectorySetpoint (e.g., hover at a certain altitude)
        self.target_altitude = -1.5  # meters (NED frame, so negative is up)
        self.hold_position = [0.0, 0.0, self.target_altitude]
        self.hold_yaw = 0  # radians

        self.home_position = [0.0, 0.0, 0.0]
        self.home_yaw = 0.0  # radians
        self.current_vehicle_position = [0.0, 0.0, 0.0]
        self.current_vehicle_yaw = 0.0
        self.current_vehicle_position_timestamp = None
        self.did_update_home_position = False
        self.home_position_update_time = None

        self.flight_duration = 100.0  # seconds
        self.flight_start_time = None

        self.flight_params = {
            "helix_height": 1,  # Height of the helix
            "helix_radius": 1,  # Radius of the helix
            "helix_num_passes": 3,  # Number of passes in the helix
            "helix_start_height": 0.2,  # Starting height of the helix
            "helix_rate": 15,  # Rate of the helix path in Hz
        }
        self.rate = 20  # Hz, rate of the helix path
        self.path = Helix(rate=self.rate)
        self.path.set_params(
            helix_height=self.flight_params["helix_height"],
            helix_radius=self.flight_params["helix_radius"],
            helix_num_passes=self.flight_params["helix_num_passes"],
            helix_start_height=self.flight_params["helix_start_height"],
            helix_rate=self.flight_params["helix_rate"]
        )
        self.start_position = self.path.get_starting_position(invert_axis='z')
        self.end_position = self.path.get_ending_position(invert_axis='z')
        self.steps = self.path.get_num_steps()
        self.hit_figure_8 = False
        self.offboard_arr_counter = 0


        self.get_logger().info(
            f"Control loop running. Attempting to arm, wait 10s, then disarm."
        )

    def helix_params_callback(self, data):
        self.get_logger().info(f"Received helix parameters: {data.data}")
        self.path = Helix(rate=self.rate)
        self.flight_params = {
            "helix_height": data.data[1],
            "helix_radius": data.data[0],
            "helix_num_passes": data.data[2],
            "helix_start_height": data.data[3],
            "helix_rate": self.flight_params["helix_rate"]
        }
        self.path.set_params(
            helix_height=self.flight_params["helix_height"],
            helix_radius=self.flight_params["helix_radius"],
            helix_num_passes=self.flight_params["helix_num_passes"],
            helix_start_height=self.flight_params["helix_start_height"],
            helix_rate=self.flight_params["helix_rate"]
        )
        self.start_position = self.path.get_starting_position(invert_axis='z')
        self.end_position = self.path.get_ending_position(invert_axis='z')
        self.steps = self.path.get_num_steps()
        self.hit_figure_8 = False
        self.offboard_arr_counter = 0
        msg = Int8()
        msg.data = 1  # Indicating parameters received
        self.state_publisher.publish(msg)

    def start_flight(self, msg: Bool) -> None:
        """Callback function to start the flight sequence."""
        self.hit_figure_8 = False
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
                


    def offboard_move_callback(self):
        if self.offboard_arr_counter < self.steps:
            state = self.path.get_state(self.offboard_arr_counter)
            position = state.get_position(invert_axis='z')
            yaw = state.get_yaw()
            velocity = state.get_velocity()
            acceleration = state.get_acceleration()
            yawspeed = state.get_yaw_rate()
            self._publish_trajectory_setpoint(
                position, yaw, velocity=velocity, acceleration=acceleration, yawspeed=yawspeed
            )   
            # self.trajectory_setpoint_publisher.publish(
            #     self.path[self.offboard_arr_counter]
            # )

        if self.offboard_arr_counter >= self.steps:
            self._publish_trajectory_setpoint(
                [self.end_position.x, self.end_position.y, self.end_position.z], 
                self.hold_yaw, velocity=[0.0, 0.0, 0.0], acceleration=[0.0, 0.0, 0.0], yawspeed=0.0
            )

        if self.offboard_arr_counter == self.steps + 100:
            self.figure8_timer.cancel()
            self.current_system_state = "LANDING"
            self.get_logger().info("Figure 8 path completed, landing now.")

        self.offboard_arr_counter += 1


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

        elif self.current_system_state == "OFFBOARD_ENGAGING":
            # Wait a short period for mode switch to take effect (e.g., 0.5s)
            if self.offboard_mode_set_time and \
               self.get_clock().now() - self.offboard_mode_set_time >= Duration(seconds=0.5):
                self._arm_vehicle()
                self.armed_time = self.get_clock().now()
                self.current_system_state = "ARMED"
                self.get_logger().info("Vehicle arm command sent.")

        elif self.current_system_state == "ARMED":
            if self.armed_time and \
               self.get_clock().now() - self.armed_time >= Duration(seconds=self.init_takeoff_hold_time):
                self.flight_start_time = self.get_clock().now()
                self.current_system_state = "FLIGHT_ENGAGED"
                self.get_logger().info("Takeoff complete. Starting flight sequence.")
            else:
                try:
                    self._publish_trajectory_setpoint([self.start_position.x, self.start_position.y, self.start_position.z], self.hold_yaw)
                except AssertionError as e:
                    self.get_logger().error(f"Error publishing trajectory setpoint: {e}")
                    self.current_system_state = "DISARMED_IDLE"
                    self.get_logger().info("Disarming due to error in trajectory setpoint.")
                    self._disarm_vehicle()
                    self.get_logger().info(f"Attempted position: {self.start_position.x}, {self.start_position.y}, {self.start_position.z}")
                    return

        elif self.current_system_state == "FLIGHT_ENGAGED":
            # This state is not used in the current sequence, but could be used for flight control.
            # For example, to maintain position or send other commands.
            if self.get_clock().now() - self.flight_start_time < Duration(seconds=self.flight_duration):
                if not self.hit_figure_8:
                    self.get_logger().info("Doing figure 8 now")
                    self.figure8_timer = self.create_timer(
                        1 / self.rate, self.offboard_move_callback
                    )
                    self.hit_figure_8 = True                

        elif self.current_system_state == "LANDING":
            self.figure8_timer.cancel()
            self._land_vehicle()
            self.current_system_state = "DISARMED_IDLE"
            msg = Int8()
            msg.data = 0  # Indicating disarmed state
            self.state_publisher.publish(msg)
            self.get_logger().info("Vehicle land command sent.")
            self.get_logger().info("Arm/disarm sequence complete. Stopping active control commands.")
            # self.destroy_node()


        elif self.current_system_state == "DISARMED_IDLE":
            # The control_timer should have been cancelled.
            # If not, something is wrong or it was restarted.
            pass

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

        # Helper to rotate a vector by a given yaw (radians)
        def rotate_vector(vec, yaw):
            if vec is None:
                return None
            if len(vec) != 3:
                raise ValueError("Vector must be length 3")
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            x, y, z = vec
            x_r = x * cos_yaw - y * sin_yaw
            y_r = x * sin_yaw + y * cos_yaw
            return [float(x_r), float(y_r), float(z)]

        # Adjust position: rotate by home_yaw, then translate by home_position
        rotated_position = rotate_vector([float(p) for p in position], self.home_yaw)
        msg.position = [rotated_position[i] + float(self.home_position[i]) for i in range(3)]

        # Adjust acceleration: rotate by home_yaw
        if acceleration is not None:
            msg.acceleration = rotate_vector(acceleration, self.home_yaw)

        # Adjust velocity: rotate by home_yaw
        if velocity is not None:
            msg.velocity = rotate_vector(velocity, self.home_yaw)

        # Adjust yaw: add home_yaw
        msg.yaw = float(yaw) + self.home_yaw

        # Adjust yawspeed: no change needed (already in body frame)
        if yawspeed is not None:
            msg.yawspeed = float(yawspeed)

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def battery_status_callback(self, msg: BatteryStatus) -> None:
        """Callback function for battery_status topic subscriber."""
        # The 'remaining' field is a float from 0.0 (empty) to 1.0 (full)
        # NAN if unknown.
        self.get_logger().info(f"Battery Remaining: {msg.remaining:.1f}%")

    def vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        """Callback function for vehicle_odometry topic subscriber."""
        # self.get_logger().info("Received Vehicle Odometry data.")
        self.current_vehicle_position = msg.position
        w, x, y, z = msg.q
        self.current_vehicle_yaw = self.get_yaw_from_quaternion(w, x, y, z)
        self.current_vehicle_position_timestamp = self.get_clock().now()
        # self.get_logger().debug("Received Vehicle Odometry data.")
        self.did_update_vehicle_position = True

        # self.get_logger().info(f"Vehicle Position Z: {vehicle_position_z:.2f}m")
        

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
            return
        elif self.get_clock().now() - self.current_vehicle_position_timestamp < Duration(seconds=0.1):
            self.home_position = self.current_vehicle_position.copy()
            self.home_yaw = self.current_vehicle_yaw
            self.get_logger().info(f"Home position updated: {self.home_position}")
            self.get_logger().info(f"Home yaw updated: {self.home_yaw} radians")
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


def main(args=None) -> None:
    """
    Main function to initialize and run the HeartbeatNode.
    """
    rclpy.init(args=args)
    heartbeat_node = HeartbeatNode()
    try:
        rclpy.spin(heartbeat_node)
    except KeyboardInterrupt:
        heartbeat_node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        heartbeat_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"An error occurred in main: {e}")