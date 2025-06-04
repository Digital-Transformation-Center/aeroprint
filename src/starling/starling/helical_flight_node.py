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
from starling.helix import Helix

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardHelicalFlightNode(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__("flight_control_node")

        self.get_logger().info("Helical Flight Control Node Alive!")

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
        self.offboard_arr_counter = 0
        self.start_time = time.time()
        self.start_altitude = 0.6
        self.end_altitude = 1.1
        self.start_height = 0.0
        self.object_height = 0.0
        self.scan_ended = False
        self.helix_params = {
            "h": 1.0,
            "r": 1.0,
            "g": 0.5
        }

    def create_path(self):
        """
        Creates a flight path consisting of multiple circular levels at different altitudes.
        This method calculates a series of altitudes for circular flight paths, starting from a maximum height
        and descending to a minimum height. The number of circular levels is determined by `num_circles`.
        The altitudes are stored in the `circle_altitudes` list and are used to initialize circular flight paths.
        
        Attributes:

            circle_altitudes (list): A list to store the altitudes for each circular level.
            num_circles (int): The number of circular levels to create.
            min_height (float): The minimum altitude for the flight path.
            max_height (float): The maximum altitude for the flight path.
            start_altitude (float): The starting altitude for the flight path.
            end_altitude (float): The ending altitude for the flight path.
        
        Logs:

            Logs the starting altitude and the list of calculated circle altitudes.
        
        Calls:

            self.init_circle(altitude): Initializes a circular flight path at the given altitude.
        """

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

    def start_height_callback(self, msg):
        """
        Callback function to update the start height.
        
        This function is triggered when a new message is received. It updates the 
        start height with the data from the message and logs the updated height.
        
        Args:

            msg: The message containing the new start height data. It is expected 
                 to have a 'data' attribute that holds the height value.
        """

        self.start_height = msg.data
        self.get_logger().info("Updating start height to " + str(msg.data))

    def object_height_callback(self, msg):
        """
        Callback function to update the object's height.

        This function is triggered when a new message is received. It updates the 
        object's height attribute with the data from the message and logs the update.
        
        Args:

            msg: The message containing the new height data. It is expected to have 
                 a 'data' attribute that holds the height value.
        """

        self.object_height = msg.data
        self.get_logger().info("Updating object height to " + str(msg.data))

    def radius_callback(self, msg):
        """
        Callback function to update the radius based on the received message.
        
        Args:

            msg (Message): The message containing the new radius value.
        
        Returns:

            None
        """

        self.radius = msg.data
        self.get_logger().info("Updating radius to " + str(msg.data))

    def ready_callback(self, msg):
        """
        Callback function that is triggered when a ready message is received.
        This function handles the initialization and reset of various components
        based on the received message. If the message indicates readiness, it 
        resets the system, logs the event, creates a flight path, and starts a 
        timer for periodic callbacks. If the message does not indicate readiness, 
        it resets the system and initiates landing.
        
        Args:

            msg (std_msgs.msg.Bool): A message indicating whether the system is ready.
        """

        b = Bool(); b.data  = False
        self.scan_start_pub.publish(b)
        self.scan_end_pub.publish(b)
        self.scan_ended = False
        if msg.data:
            self.voxl_reset.reset()
            self.reset()
            self.get_logger().info("Recieved ready command.")
            self.create_path()
            self.armed = False
            # self.publish_takeoff_setpoint(0.0, 0.0, self.end_altitude)
            self.start_time = time.time()
            
            self.timer = self.create_timer(0.1, self.timer_callback)
        else:
            self.reset()
            self.land()
        self.ready = msg.data


    def init_helix(self, altitude):
        self.helix = Helix(self.rate)
        self.helix.set_params(
            self.helix_params["h"],
            self.helix_params["r"],
            self.helix_params["g"]
        )


    def timer_callback(self) -> None:
        """
        Timer callback function that handles offboard control and flight operations.
        This function is periodically called to:
        
        - Publish offboard control heartbeat signals.
        - Engage offboard mode and arm the vehicle after a certain number of setpoints.
        - Increment the offboard setpoint counter.
        - Publish takeoff setpoints for the vehicle to reach the start altitude.
        - Initiate a scan operation and start a figure-8 flight pattern if conditions are met.
        
        The function performs the following steps:
        
        1. Publishes offboard control heartbeat signals.
        2. Engages offboard mode and arms the vehicle when the offboard setpoint counter reaches 10.
        3. Increments the offboard setpoint counter if it is less than 11.
        4. Publishes takeoff setpoints until 10 seconds have passed since the start time.
        5. Initiates a scan operation and starts a figure-8 flight pattern if the vehicle is ready and the figure-8 pattern has not been started.
        
        Returns:

            None
        """

        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.armed = True

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        
        if self.start_time + 10 > time.time():
            self.get_logger().info("Taking off to " + str(self.start_altitude))
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

    def reset(self):
        """
        Resets the flight control node to its initial state.

        This method performs the following actions:

        - Attempts to cancel the current timer. Logs a message if it fails.
        - Attempts to cancel the figure-8 timer. Logs a message if it fails.
        - Sets the scan_ended flag to False.
        - Clears the flight path.
        - Resets the offboard setpoint counter.
        - Sets the hit_figure_8 flag to False.
        - Sets the taken_off flag to False.
        - Sets the armed flag to False.
        - Resets the offboard arrival counter.
        - Logs a message indicating that the flight control node has been reset.
        """

        try:
            self.timer.cancel()
        except: 
            self.get_logger().info("Failed to cancel timer.")
        try:
            self.figure8_timer.cancel()
        except: 
            self.get_logger().info("Failed to cancel fig timer.")
        self.scan_ended = False
        self.path = []
        self.offboard_setpoint_counter = 0
        self.hit_figure_8 = False
        self.taken_off = False
        self.armed = False
        self.offboard_arr_counter = 0
        # self.clear_trajectory()
        self.get_logger().info("Reset Flight Control Node.")
    
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
        self.reset()
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")
        
        # self.hit_figure_8 = False

    def offboard_move_callback(self):
        """
        Callback function to handle offboard movement.
        This function is called periodically to publish the next setpoint in the 
        offboard path. It also handles the end of the scan by publishing a 
        scan end message and initiating a landing sequence after a certain 
        number of iterations.

        Behavior:

        - Publishes the next trajectory setpoint if the counter is within the path length.
        - If the counter exceeds the path length and the scan has not ended, it logs the end of the scan,
          publishes a scan end message, and sets the scan ended flag.
        - Publishes a takeoff setpoint with the end altitude.
        - Initiates landing if the counter exceeds the path length by 100.
        
        Attributes:
        
        - self.offboard_arr_counter (int): Counter to track the current position in the path.
        - self.path (list): List of trajectory setpoints.
        - self.scan_ended (bool): Flag to indicate if the scan has ended.
        - self.end_altitude (float): Altitude to use for the takeoff setpoint at the end of the scan.
        - self.trajectory_setpoint_publisher (Publisher): Publisher for trajectory setpoints.
        - self.scan_end_pub (Publisher): Publisher for scan end messages.
        - self.get_logger (function): Function to get the logger instance.
        - self.publish_takeoff_setpoint (function): Function to publish the takeoff setpoint.
        - self.land (function): Function to initiate landing.
        """

        # if self.offboard_arr_counter < self.helix.get_num_steps():
        #     self.trajectory_setpoint_publisher.publish(
        #         next_state = self.helix.get_state(self.offboard_arr_counter),
        #         msg = TrajectorySetpoint()
        #     )
        #     msg.position = [next_state.x, next_state.y, next_state.z]
        #     msg.velocity = [next_state.vx, next_state.vy, next_state.vz]
        #     msg.acceleration = [next_state.ax, next_state.ay, next_state.az]
        #     msg.yaw = next_state.yaw
        #     msg.yaw_rate = next_state.yaw_rate
        #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        #     self.trajectory_setpoint_publisher.publish(msg)
        # )

        if self.offboard_arr_counter >= len(self.path):
            if not self.scan_ended:
                self.get_logger().info("End of Scan.")
                b = Bool(); b.data  = True
                self.scan_end_pub.publish(b)
                self.scan_ended = True
            self.publish_takeoff_setpoint(0.0, 0.0, -self.end_altitude)

        if self.offboard_arr_counter == len(self.path) + 100:
            self.land()

        self.offboard_arr_counter += 1

    def publish_takeoff_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        
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

    def clear_trajectory(self):
        """Clear the trajectory."""
        empty_msg = TrajectorySetpoint()
        self.trajectory_setpoint_publisher.publish(empty_msg)

import subprocess

class VOXLQVIOController():
    """
    Controller class for handling VOXL QVIO (Visual-Inertial Odometry) operations.
    """

    def __init__(self) -> None:
        None

    def reset(self):
        """
        Sends a reset command to the VOXL QVIO system.
        """
        try:
            subprocess.run(["voxl-reset-qvio"])
            return True
        except Exception as e:
            print(f"Error sending VIO reset command: {e}")
            return False

def main(args=None) -> None:
    """
    Main function to initialize the ROS node and start the flight control node.
    """
    rclpy.init(args=args)
    offboard_figure8_node = OffboardStarlingNode()
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
