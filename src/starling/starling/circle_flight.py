#Spring 2025 circle path with pauses




#!/usr/bin/env python3
"""
circle_flight.py: ROS node to perform flight based on scan parameters.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle, Timothy Marshall"
__email__ = "ryan.kuederle@udri.udayton.edu "
__version__ = "0.1.0"
__status__ = "Beta"




#!/usr/bin/env python3

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Float32
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)

class FlightControlNode(Node):
    """
    Subscribes to user parameters from GUI:
      - /host/gui/out/radius (float)
      - /host/gui/out/object_height (float)
      - /host/gui/out/start_height (float)
      - /host/gui/out/offset_x (float)
      - /host/gui/out/offset_y (float)
      - /host/gui/out/ready (bool)

    Generates a circle around (offset_x, offset_y)
    with radius 'radius'. Then commands a flight in offboard mode.
    """

    def __init__(self):
        super().__init__("flight_control_node")
        self.get_logger().info("FlightControlNode started!")

        # Publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile
        )

        # Subscribers from GUI
        self.radius_sub = self.create_subscription(
            Float32, "/host/gui/out/radius", self.radius_callback, 10
        )
        self.object_height_sub = self.create_subscription(
            Float32, "/host/gui/out/object_height", self.object_height_callback, 10
        )
        self.start_height_sub = self.create_subscription(
            Float32, "/host/gui/out/start_height", self.start_height_callback, 10
        )
        self.offset_x_sub = self.create_subscription(
            Float32, "/host/gui/out/offset_x", self.offset_x_callback, 10
        )
        self.offset_y_sub = self.create_subscription(
            Float32, "/host/gui/out/offset_y", self.offset_y_callback, 10
        )
        self.ready_sub = self.create_subscription(
            Bool, "/host/gui/out/ready", self.ready_callback, 10
        )

        # Internal state
        self.radius = 0.0
        self.object_height = 0.0
        self.start_height = 0.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.ready = False

        self.rate = 20  # Hz
        self.offboard_setpoint_counter = 0
        self.cycle_s = 20.0  # circle time in seconds

        self.path = []
        self.path_index = 0
        self.flight_timer = None
        self.offboard_timer = self.create_timer(0.1, self.publish_offboard_heartbeat)
        self.armed = False

        self.get_logger().info("FlightControlNode setup complete.")

    # ------------------------------------------------------------------
    # GUI Subscriptions
    # ------------------------------------------------------------------
    def radius_callback(self, msg: Float32):
        self.radius = msg.data
        self.get_logger().info(f"Radius updated: {self.radius:.2f}")

    def object_height_callback(self, msg: Float32):
        self.object_height = msg.data
        self.get_logger().info(f"Object height updated: {self.object_height:.2f}")

    def start_height_callback(self, msg: Float32):
        self.start_height = msg.data
        self.get_logger().info(f"Start height updated: {self.start_height:.2f}")

    def offset_x_callback(self, msg: Float32):
        self.offset_x = msg.data
        self.get_logger().info(f"Offset X updated: {self.offset_x:.2f}")

    def offset_y_callback(self, msg: Float32):
        self.offset_y = msg.data
        self.get_logger().info(f"Offset Y updated: {self.offset_y:.2f}")

    def ready_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Ready=TRUE => Starting flight path.")
            self.ready = True

            if self.flight_timer:
                self.flight_timer.cancel()

            self.build_path()  # build path from user inputs
            self.path_index = 0
            self.offboard_setpoint_counter = 0

            # Start flight timer
            self.flight_timer = self.create_timer(
                1.0 / self.rate, self.flight_callback
            )
        else:
            self.get_logger().info("Ready=FALSE => Land / stop flight.")
            self.ready = False
            if self.flight_timer:
                self.flight_timer.cancel()
            self.path = []
            self.land()

    def vehicle_status_callback(self, vehicle_status):
        # optional for debugging
        pass

    # ------------------------------------------------------------------
    # Path Generation
    # ------------------------------------------------------------------
    def build_path(self):
        """
        Builds a circle around (offset_x, offset_y) with radius = self.radius,
        at altitude = -(start_height + object_height).
        """
        self.path = []
        steps = int(self.cycle_s * self.rate)
        # define altitude: negative for NED
        altitude_ned = -(self.start_height + self.object_height)
        if altitude_ned > -0.3:
            # ensure at least 0.3m if the user input is too small
            altitude_ned = -0.3

        dadt = (2.0 * math.pi) / self.cycle_s

        for i in range(steps):
            theta = 2.0 * math.pi * (i / steps)
            x = self.offset_x + self.radius * math.cos(theta)
            y = self.offset_y + self.radius * math.sin(theta)
            vx = dadt * -self.radius * math.sin(theta)
            vy = dadt *  self.radius * math.cos(theta)

            sp = TrajectorySetpoint()
            sp.position = [x, y, altitude_ned]
            sp.velocity = [vx, vy, 0.0]
            sp.yaw = math.atan2(vy, vx)
            self.path.append(sp)

        self.get_logger().info(
            f"Built circle path => {len(self.path)} points."
            f" Center=({self.offset_x:.2f},{self.offset_y:.2f}),"
            f" radius={self.radius:.2f}, alt={-altitude_ned:.2f}m"
        )

    # ------------------------------------------------------------------
    # Offboard Publishing
    # ------------------------------------------------------------------
    def publish_offboard_heartbeat(self):
        """
        Publish OffboardControlMode at 10Hz to keep PX4 in offboard mode
        """
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

    def flight_callback(self):
        """
        Runs at self.rate Hz once user is 'ready'.
        1) Provide ~10 setpoints => offboard transition
        2) Arm
        3) Step through path
        4) Land
        """
        if self.offboard_setpoint_counter < 10:
            # Publish a simple takeoff setpoint at ~1m altitude
            self.publish_takeoff_setpoint(0.0, 0.0, -1.0)
            self.offboard_setpoint_counter += 1
            return
        elif self.offboard_setpoint_counter == 10:
            # Switch to offboard mode + arm
            self.engage_offboard_mode()
            self.arm()
            self.offboard_setpoint_counter += 1
            return

        # Follow path
        if self.path_index < len(self.path):
            sp = self.path[self.path_index]
            sp.timestamp = self.get_clock().now().nanoseconds // 1000
            self.trajectory_setpoint_pub.publish(sp)
            self.path_index += 1
        else:
            self.get_logger().info("Completed path => Land.")
            self.land()
            if self.flight_timer:
                self.flight_timer.cancel()

    # ------------------------------------------------------------------
    # PX4 Command Helpers
    # ------------------------------------------------------------------
    def publish_takeoff_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_pub.publish(msg)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )
        self.get_logger().info("Switching to OFFBOARD mode.")

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )
        self.get_logger().info("Arm command sent.")

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )
        self.get_logger().info("Disarm command sent.")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Land command sent.")

    def publish_vehicle_command(self, cmd, **params):
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FlightControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.land()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()




