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
            "/starling/out/fc/scan_end" ,
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
        # self.init_circle(self.start_altitude)
        # self.init_circle(self.end_altitude)

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
        self.get_logger().info("circle altitudes: "+ str(circle_altitudes))
        for altitude in circle_altitudes:
            self.init_circle(-altitude)
        
        # self.init_circle(-self.start_altitude)
        # self.init_circle(-self.end_altitude)

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
        b = Bool(); b.data  = False
        self.scan_start_pub.publish(b)
        self.scan_end_pub.publish(b)
        self.scan_ended = False
        if msg.data:
            self.publish_offboard_control_heartbeat_signal()
            self.get_logger().info("Recieved ready command.")
            self.create_path()
            self.engage_offboard_mode()
            self.arm()
            self.armed = True
            # self.publish_takeoff_setpoint(0.0, 0.0, self.end_altitude)
            self.start_time = time.time()
            self.offboard_setpoint_counter
            self.timer = self.create_timer(0.1, self.timer_callback)
        else:
            try:
                self.timer.cancel()
            except: pass
            self.offboard_arr_counter = 0
            self.path = []
            self.land()
            self.hit_figure_8 = False

        self.ready = msg.data


    def init_circle(self, altitude):

        dt = 1.0 / self.rate
        dadt = (2.0 * math.pi) / self.cycle_s
        r = self.radius

        for i in range(self.steps):
            msg = TrajectorySetpoint()

            a = (-math.pi) + i * (2.0 * math.pi / self.steps)
            

            msg.position = [r + r * math.cos(a), r * math.sin(a), altitude]
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

        for i in range(self.steps):
            next_yaw = self.path[(i + 1) % self.steps].yaw
            curr = self.path[i].yaw
            if next_yaw - curr < -math.pi:
                next_yaw += 2.0 * math.pi
            if next_yaw - curr > math.pi:
                next_yaw -= 2.0 * math.pi

            self.path[i].yawspeed = (next_yaw - curr) / dt

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.armed = True

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        
        if self.start_time + 10 > time.time():
            self.publish_takeoff_setpoint(0.0, 0.0, -self.start_altitude)
        else:
            if not self.hit_figure_8 and self.ready:
                self.get_logger().info("Starting Scan Now.")
                b = Bool(); b.data = True
                self.scan_start_pub.publish(b)
                self.figure8_timer = self.create_timer(
                    1 / self.rate, self.offboard_move_callback
                )
                self.hit_figure_8 = True

    def vehicle_local_position_callback(self, vehicle_local_position):
        print(vehicle_local_position)
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

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
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")
        self.taken_off = False
        self.path = []
        # self.hit_figure_8 = False

    def offboard_move_callback(self):
        if self.offboard_arr_counter < len(self.path):
            self.trajectory_setpoint_publisher.publish(
                self.path[self.offboard_arr_counter]
            )

        if self.offboard_arr_counter >= len(self.path):
            if not self.scan_ended:
                self.get_logger().info("End of Scan.")
                b = Bool(); b.data  = True
                self.scan_end_pub.publish(b)
                self.scan_ended = True
            self.publish_takeoff_setpoint(0.0, 0.0, -self.end_altitude)

        if self.offboard_arr_counter == len(self.path) + 100:
            self.figure8_timer.cancel()
            self.land()

        self.offboard_arr_counter += 1

    def publish_takeoff_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.00
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
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

import subprocess

class VOXLQVIOController():
    def __init__(self) -> None:
        None

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
