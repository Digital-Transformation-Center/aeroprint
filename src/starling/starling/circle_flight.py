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
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_system_default
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

        self.voxl_reset = VOXLQVIOController()
        self.voxl_reset.reset()
        self.rate = 20
        self.radius = 0.9
        self.cycle_s = 40
        
        self.steps = self.cycle_s * self.rate
        self.path = []
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.taken_off = False
        self.hit_figure_8 = False
        self.armed = False
        self.offboard_setpoint_counter = 0
        self.start_time = time.time()
        self.offboard_arr_counter = 0
        self.start_altitude = 0.6
        self.end_altitude = 1.1
        self.start_height = 0.0
        self.object_height = 0.0
        self.scan_ended = False

    def create_path(self):
        # This is very extra right now, but makes it easier to add levels.
        circle_altitudes = []
        num_circles = 3
        min_height = self.start_height + 0.20
        max_height = self.start_height + self.object_height + 0.2
        self.start_altitude = max_height
        self.end_altitude = min_height
        self.get_logger().info("Flying path from " + str(self.start_altitude) + "m.")
        for lev in range(num_circles):
            if lev == 0:
                circle_altitudes.append(max_height)
            elif lev == num_circles - 1:
                circle_altitudes.append(min_height)
            else:
                inter_lev = max_height - ((lev) * ((max_height - min_height) / (num_circles - 1)))
                circle_altitudes.append(inter_lev)
        self.get_logger().info("circle altitudes: " + str(circle_altitudes))
        for altitude in circle_altitudes:
            self.init_circle(-altitude)
        
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
        self.voxl_reset.reset()
        b = Bool()
        b.data = False
        self.scan_start_pub.publish(b)
        self.scan_end_pub.publish(b)
        self.scan_ended = False
        if msg.data:
            self.publish_offboard_control_heartbeat_signal()
            self.get_logger().info("Received ready command.")
            self.create_path()
            self.engage_offboard_mode()
            self.arm()
            self.armed = True
            self.start_time = time.time()
            self.publish_offboard_control_heartbeat_signal()
            self.publish_takeoff_setpoint(0.0, 0.0, -self.start_altitude)
            time.sleep(5)
            self.figure8_timer = self.create_timer(1.0 / self.rate, self.offboard_move_callback)

    def offboard_move_callback(self):
        if self.offboard_arr_counter < len(self.path):
            self.trajectory_setpoint_publisher.publish(self.path[self.offboard_arr_counter])

        if self.offboard_arr_counter >= len(self.path):
            if not self.scan_ended:
                self.get_logger().info("End of half-circle path.")
                b = Bool()
                b.data = True
                self.scan_end_pub.publish(b)
                self.scan_ended = True
            self.publish_takeoff_setpoint(0.0, 0.0, -self.end_altitude)

        if self.offboard_arr_counter == len(self.path) + 100:
            self.figure8_timer.cancel()
            self.land()

        self.offboard_arr_counter += 1

    def init_circle(self, altitude, num_stops=8, pause_duration=5.0):
        """Initialize circle trajectory with stops at specified intervals."""
        dt = 1.0 / self.rate
        dadt = (2.0 * math.pi) / self.cycle_s
        r = self.radius

        # Calculate the angle interval for stops
        stop_interval = self.steps // num_stops
        stop_angles = [i * (2.0 * math.pi / num_stops) for i in range(num_stops)]

        for i in range(self.steps):
            msg = TrajectorySetpoint()

            # Define angle a
            a = (i * (2.0 * math.pi) / self.steps) - math.pi / 2

            msg.position = [r * math.cos(a), r * math.sin(a), altitude]
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
            msg.yaw = math.atan2(msg.acceleration[1], msg.acceleration[0])

            self.path.append(msg)

            # Insert pauses at specified stop angles
            if i % stop_interval == 0 and i != 0:
                # Add pause setpoints
                for _ in range(int(pause_duration * self.rate)):
                    pause_msg = TrajectorySetpoint()
                    pause_msg.position = msg.position
                    pause_msg.velocity = [0.0, 0.0, 0.0]
                    pause_msg.acceleration = [0.0, 0.0, 0.0]
                    pause_msg.yaw = msg.yaw
                    pause_msg.yawspeed = 0.0
                    self.path.append(pause_msg)

        # Calculate yawspeed for smooth rotation
        for i in range(len(self.path) - 1):
            next_yaw = self.path[i + 1].yaw
            curr = self.path[i].yaw
            if next_yaw - curr < -math.pi:
                next_yaw += 2.0 * math.pi
            if next_yaw - curr > math.pi:
                next_yaw -= 2.0 * math.pi

            self.path[i].yawspeed = (next_yaw - curr) / dt

        # Set yawspeed for the last point
        self.path[-1].yawspeed = 0.0

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_takeoff_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [0.0, 0.0, 0.0]
        self.trajectory_setpoint_publisher.publish(msg)

    def engage_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = float(1)  # base mode
        msg.param2 = float(6)  # custom main mode
        self.vehicle_command_publisher.publish(msg)r

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = float(1)
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info("Arm command sent")

    def land(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info("Land command sent")

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg


class VOXLQVIOController:
    def __init__(self):
        self.voxl_services = [
            "qvio_server",
            "voa",
            "vislam",
            "msckf",
            "vio",
            "voxl_mpa_to_ros",
        ]

    def reset(self):
        for srv in self.voxl_services:
            print(f"Restarting {srv}...")
            self.restart_service(srv)

    def restart_service(self, service_name):
        print(f"Service {service_name} restarted.")


def main(args=None):
    rclpy.init(args=args)
    offboard_control_node = OffboardFigure8Node()
    rclpy.spin(offboard_control_node)
    offboard_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
