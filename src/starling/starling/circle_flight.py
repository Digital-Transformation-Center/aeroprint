#Spring 2025 circle path with pauses




#!/usr/bin/env python3
"""
circle_flight.py: ROS node to perform flight based on scan parameters.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle, Timothy Marshall"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.1.0"
__status__ = "Beta"





import rclpy
import math
import time
import subprocess
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    qos_profile_system_default,
)
from std_msgs.msg import Bool, Float32

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardFigure8Node(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__("flight_control_node")

        self.get_logger().info("Starling Flight Control Node Alive!")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create flight publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile,
        )

        # Create integrations pubs & subs
        self.ready_sub = self.create_subscription(
            Bool,
            "/host/gui/out/ready",
            self.ready_callback,
            qos_profile_system_default
        )
        self.radius_sub = self.create_subscription(
            Float32,
            "/host/gui/out/radius",
            self.radius_callback,
            qos_profile_system_default
        )
        self.object_height_sub = self.create_subscription(
            Float32,
            "/host/gui/out/object_height",
            self.object_height_callback,
            qos_profile_system_default
        )
        self.start_height_sub = self.create_subscription(
            Float32,
            "/host/gui/out/start_height",
            self.start_height_callback,
            qos_profile_system_default
        )
        self.scan_start_pub = self.create_publisher(
            Bool,
            "/starling/out/fc/scan_start",
            qos_profile_system_default
        )
        self.scan_end_pub = self.create_publisher(
            Bool,
            "/starling/out/fc/scan_end",
            qos_profile_system_default
        )

        self.ready = False

        # For resetting VIO (VOXL)
        self.voxl_reset = VOXLQVIOController()
        self.voxl_reset.reset()

        # Flight/Path parameters
        self.rate = 20
        self.radius = 0.0         # Distance from center
        self.cycle_s = 20         # Duration for one circle (seconds)
        self.steps = int(self.cycle_s * self.rate)

        # State variables
        self.path = []
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.taken_off = False
        self.hit_figure_8 = False
        self.armed = False
        self.offboard_setpoint_counter = 0
        self.start_time = time.time()
        self.offboard_arr_counter = 0

        # Altitude parameters (NED: negative is up)
        self.start_altitude = 0.6
        self.end_altitude = 1.1
        self.start_height = 0.0
        self.object_height = 0.0
        self.scan_ended = False

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    def start_height_callback(self, msg):
        self.start_height = msg.data
        self.get_logger().info("Updating start height to " + str(msg.data))

    def object_height_callback(self, msg):
        self.object_height = msg.data
        self.get_logger().info("Updating object height to " + str(msg.data))

    def radius_callback(self, msg):
        self.radius = msg.data
        self.get_logger().info("Updating radius to " + str(msg.data))

    def ready_callback(self, msg):
        # Reset VOXL QVIO
        self.voxl_reset.reset()

        # Reset scan flags
        b = Bool()
        b.data = False
        self.scan_start_pub.publish(b)
        self.scan_end_pub.publish(b)
        self.scan_ended = False

        if msg.data:
            # We're "ready": start the entire offboard process
            self.publish_offboard_control_heartbeat_signal()
            self.get_logger().info("Received ready command.")
            self.create_path()
            self.engage_offboard_mode()
            self.arm()
            self.armed = True

            self.start_time = time.time()
            self.offboard_setpoint_counter = 0
            self.timer = self.create_timer(0.1, self.timer_callback)
        else:
            # Cancel flight timers and land if "un-ready"
            try:
                self.timer.cancel()
            except:
                pass
            try:
                self.figure8_timer.cancel()
            except:
                pass

            self.offboard_arr_counter = 0
            self.path = []
            self.land()
            self.hit_figure_8 = False

        self.ready = msg.data

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    # -------------------------------------------------------------------------
    # Main Flight Logic
    # -------------------------------------------------------------------------
    def create_path(self):
        """
        Generate multiple circles at different altitudes if needed.
        For example, 2 altitude levels: top -> bottom.
        """
        circle_altitudes = []
        num_circles = 2  # You can increase for more altitude layers

        # Compute top & bottom altitudes
        min_height = self.start_height + 0.20
        max_height = self.start_height + self.object_height + 0.2
        self.start_altitude = max_height
        self.end_altitude = min_height

        self.get_logger().info("Flying path from " + str(self.start_altitude) + " m downward.")

        # Fill altitude array
        for lev in range(num_circles):
            if lev == 0:
                circle_altitudes.append(max_height)
            elif lev == num_circles - 1:
                circle_altitudes.append(min_height)
            else:
                inter_lev = max_height - (
                    (lev) * ((max_height - min_height) / (num_circles - 1))
                )
                circle_altitudes.append(inter_lev)

        self.get_logger().info("circle altitudes: " + str(circle_altitudes))

        # Build the path: circle for each altitude
        for alt in circle_altitudes:
            self.init_circle(-alt)  # negative: NED coords

    def init_circle(self, altitude, num_stops=6, pause_duration=3.0):
        """
        Initialize a circle around (0,0) with radius = self.radius,
        starting at (0, +r).
        Insert 'num_stops' equally spaced hover pauses, each lasting 'pause_duration' seconds.
        """
        dt = 1.0 / self.rate
        dadt = (2.0 * math.pi) / self.cycle_s
        r = self.radius

        # We'll start the circle at angle π/2, so first waypoint is (0, r).
        # Then proceed 2π from there.
        stop_interval = int(self.steps // num_stops)

        for i in range(self.steps):
            msg = TrajectorySetpoint()

            # Angle from π/2 -> π/2 + 2π
            a = (math.pi / 2) + i * (2.0 * math.pi / self.steps)

            x = r * math.cos(a)
            y = r * math.sin(a)

            msg.position = [x, y, altitude]
            # Velocity/acceleration for a circular path
            msg.velocity = [
                dadt * -r * math.sin(a),
                dadt * r * math.cos(a),
                0.0,
            ]
            msg.acceleration = [
                dadt * -r * math.cos(a),
                dadt * -r * math.sin(a),
                0.0,
            ]
            msg.yaw = math.atan2(msg.velocity[1], msg.velocity[0])

            self.path.append(msg)

            # Insert a pause every stop_interval steps (excluding i=0)
            if i % stop_interval == 0 and i != 0:
                for _ in range(int(pause_duration * self.rate)):
                    pause_msg = TrajectorySetpoint()
                    pause_msg.position = msg.position
                    pause_msg.velocity = [0.0, 0.0, 0.0]
                    pause_msg.acceleration = [0.0, 0.0, 0.0]
                    pause_msg.yaw = msg.yaw
                    pause_msg.yawspeed = 0.0
                    self.path.append(pause_msg)

        # Smooth out yawspeed across all generated waypoints
        for i in range(len(self.path) - 1):
            curr_yaw = self.path[i].yaw
            next_yaw = self.path[i + 1].yaw
            # unwrap
            if next_yaw - curr_yaw < -math.pi:
                next_yaw += 2.0 * math.pi
            if next_yaw - curr_yaw > math.pi:
                next_yaw -= 2.0 * math.pi

            self.path[i].yawspeed = (next_yaw - curr_yaw) / dt

        # Last waypoint yawspeed
        if self.path:
            self.path[-1].yawspeed = 0.0

    def timer_callback(self) -> None:
        """
        Runs at 0.1s intervals (10 Hz).
        - Publishes offboard heartbeat signals.
        - Handles the initial 10-second takeoff.
        - After 10s, we start the scanning path.
        """
        self.publish_offboard_control_heartbeat_signal()

        # Give PX4 a few setpoints before offboard mode
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.armed = True

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        # For the first 10s, ascend vertically above (0,r).
        if time.time() < self.start_time + 10.0:
            self.publish_takeoff_setpoint(0.0, self.radius, -self.start_altitude)
        else:
            # Once 10s is up, begin the circle flight if not already started.
            if not self.hit_figure_8 and self.ready:
                self.get_logger().info("Starting Scan Now.")
                b = Bool()
                b.data = True
                self.scan_start_pub.publish(b)
                self.figure8_timer = self.create_timer(
                    1.0 / self.rate, self.offboard_move_callback
                )
                self.hit_figure_8 = True

    def offboard_move_callback(self):
        """
        Publishes waypoints from self.path at self.rate Hz.
        Once path is done, returns to (0, r), lands.
        """
        if self.offboard_arr_counter < len(self.path):
            self.trajectory_setpoint_publisher.publish(
                self.path[self.offboard_arr_counter]
            )
        else:
            # Once we've exhausted all path points:
            if not self.scan_ended:
                self.get_logger().info("End of Scan.")
                b = Bool()
                b.data = True
                self.scan_end_pub.publish(b)
                self.scan_ended = True

            # Return to (0, r) at end_altitude
            self.publish_takeoff_setpoint(0.0, self.radius, -self.end_altitude)

        # After waiting ~100 cycles, we land
        if self.offboard_arr_counter == len(self.path) + 100:
            self.figure8_timer.cancel()
            self.land()

        self.offboard_arr_counter += 1

    # -------------------------------------------------------------------------
    # PX4 Command Methods
    # -------------------------------------------------------------------------
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode and clear path."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")
        self.taken_off = False
        self.path = []

    def publish_takeoff_setpoint(self, x: float, y: float, z: float):
        """Publish a single TrajectorySetpoint for takeoff/hover."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.00
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode at ~10 Hz."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command with optional param overrides."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


class VOXLQVIOController():
    def __init__(self) -> None:
        pass

    def reset(self):
        try:
            subprocess.run(["voxl-reset-qvio"])
            return True
        except Exception as e:
            print(f"Error sending VIO reset command: {e}")
            return False


def main(args=None) -> None:
    rclpy.init(args=args)
    offboard_figure8_node = OffboardFigure8Node()

    try:
        rclpy.spin(offboard_figure8_node)
    except KeyboardInterrupt:
        offboard_figure8_node.land()
        offboard_figure8_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)



