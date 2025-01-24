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

import subprocess

class VOXLQVIOController():
    def __init__(self) -> None:
        pass

    def reset(self):
        """Example to reset VIO from a VOXL-based system."""
        try:
            subprocess.run(["voxl-reset-qvio"])
            return True
        except Exception as e:
            print(f"Error sending VIO reset command: {e}")
            return False


class OffboardFigure8Node(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__("flight_control_node")

        self.get_logger().info("Starling Flight Control Node Alive!")

        # Configure QoS profile
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

        # Internal state
        self.ready = False
        self.scan_ended = False
        self.voxl_reset = VOXLQVIOController()
        self.voxl_reset.reset()

        # Flight / path parameters
        self.rate = 20
        self.radius = 0.0
        self.cycle_s = 30  # one circle time (seconds)
        self.steps = self.cycle_s * self.rate
        self.path = []
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.hit_figure_8 = False
        self.armed = False
        self.offboard_setpoint_counter = 0
        self.start_time = time.time()
        self.offboard_arr_counter = 0

        # Default altitudes
        self.start_altitude = 0.6
        self.end_altitude = 1.1
        self.start_height = 0.0
        self.object_height = 0.0

    def create_path(self):
        """
        Build the entire path for your flight.
        In this example, we do multiple 'circles' at different altitudes.
        """
        self.path = []  # Reset path
        # Decide top and bottom altitudes based on object height, etc.
        num_circles = 2
        min_height = self.start_height + 0.20
        max_height = self.start_height + self.object_height + 0.20
        self.start_altitude = max_height
        self.end_altitude = min_height

        circle_altitudes = []
        for lev in range(num_circles):
            if lev == 0:
                circle_altitudes.append(max_height)
            elif lev == num_circles - 1:
                circle_altitudes.append(min_height)
            else:
                # If you wanted more intermediate levels, you'd compute them here
                inter_lev = max_height - (lev) * ((max_height - min_height) / (num_circles - 1))
                circle_altitudes.append(inter_lev)

        for altitude in circle_altitudes:
            # Because PX4 uses Z negative down, we pass -altitude
            self.init_circle(-altitude)

    def init_circle(self, 
                    altitude, 
                    num_stops=4, 
                    pause_in_time=1.0, 
                    pause_center_time=1.0, 
                    pause_out_time=1.0):
        """
        Initialize circle trajectory with 'stops' at intervals around the circle.
        Instead of a pure pause, we do a radial in-and-out maneuver (closer to center).
        
        Now, we will have the drone face the center of the circle by setting:
        msg.yaw = a + math.pi
        (since the circle is centered around (r,0) ).
        """
        dt = 1.0 / self.rate
        dadt = (2.0 * math.pi) / self.cycle_s  # derivative of angle wrt time
        r = self.radius

        # We'll do `num_stops` around the circle
        stop_interval = self.steps // num_stops

        # Build the main circle path
        circle_points = []
        for i in range(self.steps):
            # angle around circle
            a = -math.pi + i * (2.0 * math.pi / self.steps)

            # Position on the circle (centered at (r, 0))
            x = r + r * math.cos(a)
            y = r * math.sin(a)

            msg = TrajectorySetpoint()
            msg.position = [x, y, altitude]

            # We'll keep velocity/acceleration purely for feedforward if desired,
            # but the main difference is that we force yaw to face center.
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

            # Force the yaw to face the center => a + pi
            msg.yaw = a + math.pi
            msg.yawspeed = 0.0  # We'll keep zero or can compute derivative if you want

            circle_points.append(msg)

        # Now insert the radial in-and-out sequences at each "stop"
        for i in range(self.steps):
            # Always push the nominal circle point
            self.path.append(circle_points[i])

            # Check if we should do the in-out motion at this step
            if i != 0 and (i % stop_interval) == 0:
                # We'll keep the same angle `a` as circle_points[i]
                # so we know the direction from the center
                cp = circle_points[i]
                angle_a = -math.pi + i * (2.0 * math.pi / self.steps)
                
                # We'll define an inner radius (half, for example)
                r_in = 0.5 * r  
                current_yaw = cp.yaw  # same angle to keep facing center

                # 1) Move inward
                steps_in = int(pause_in_time * self.rate)
                for step_j in range(steps_in):
                    frac = float(step_j) / float(max(steps_in - 1, 1))
                    # linear interpolation from r to r_in
                    s = r - frac * (r - r_in)
                    x_in = r + s * math.cos(angle_a)
                    y_in = s * math.sin(angle_a)

                    msg_in = TrajectorySetpoint()
                    msg_in.position = [x_in, y_in, cp.position[2]]
                    msg_in.velocity = [0.0, 0.0, 0.0]
                    msg_in.acceleration = [0.0, 0.0, 0.0]
                    # Keep yaw facing the center
                    msg_in.yaw = current_yaw
                    msg_in.yawspeed = 0.0
                    self.path.append(msg_in)

                # 2) Pause briefly at the inner radius
                steps_pause = int(pause_center_time * self.rate)
                x_pause = r + r_in * math.cos(angle_a)
                y_pause = r_in * math.sin(angle_a)
                for _ in range(steps_pause):
                    msg_pause = TrajectorySetpoint()
                    msg_pause.position = [x_pause, y_pause, cp.position[2]]
                    msg_pause.velocity = [0.0, 0.0, 0.0]
                    msg_pause.acceleration = [0.0, 0.0, 0.0]
                    msg_pause.yaw = current_yaw
                    msg_pause.yawspeed = 0.0
                    self.path.append(msg_pause)

                # 3) Move outward
                steps_out = int(pause_out_time * self.rate)
                for step_j in range(steps_out):
                    frac = float(step_j) / float(max(steps_out - 1, 1))
                    # linear interpolation from r_in to r
                    s = r_in + frac * (r - r_in)
                    x_out = r + s * math.cos(angle_a)
                    y_out = s * math.sin(angle_a)

                    msg_out = TrajectorySetpoint()
                    msg_out.position = [x_out, y_out, cp.position[2]]
                    msg_out.velocity = [0.0, 0.0, 0.0]
                    msg_out.acceleration = [0.0, 0.0, 0.0]
                    msg_out.yaw = current_yaw
                    msg_out.yawspeed = 0.0
                    self.path.append(msg_out)

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
        # Reset relevant state
        self.voxl_reset.reset()
        self.scan_ended = False
        b_false = Bool(); b_false.data = False
        self.scan_start_pub.publish(b_false)
        self.scan_end_pub.publish(b_false)

        if msg.data:
            self.get_logger().info("Received ready command.")
            self.create_path()  # Build the entire flight path
            self.publish_offboard_control_heartbeat_signal()
            self.engage_offboard_mode()
            self.arm()
            self.armed = True

            self.start_time = time.time()
            self.offboard_setpoint_counter = 0
            # Start a timer to send setpoints
            self.timer = self.create_timer(0.1, self.timer_callback)
        else:
            # Stop if we are flying
            try:
                self.timer.cancel()
            except:
                pass
            self.offboard_arr_counter = 0
            self.path = []
            self.land()
            self.hit_figure_8 = False

        self.ready = msg.data

    def timer_callback(self) -> None:
        """Timer callback to handle initial “takeoff” setpoint, then launch into path flight."""
        self.publish_offboard_control_heartbeat_signal()

        # Send heartbeats for a short while to ensure offboard is engaged
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.armed = True

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        # For ~10s, just hold at the takeoff setpoint
        if (time.time() - self.start_time) < 10:
            self.publish_takeoff_setpoint(0.0, 0.0, -self.start_altitude)
        else:
            # Once done, if we haven't started the flight path yet, do so
            if not self.hit_figure_8 and self.ready:
                self.get_logger().info("Starting Scan Now.")
                b = Bool()
                b.data = True
                self.scan_start_pub.publish(b)

                # Now begin publishing the path
                self.figure8_timer = self.create_timer(1.0 / self.rate, self.offboard_move_callback)
                self.hit_figure_8 = True

    def offboard_move_callback(self):
        """Publish each setpoint in the precomputed path."""
        if self.offboard_arr_counter < len(self.path):
            self.trajectory_setpoint_publisher.publish(self.path[self.offboard_arr_counter])
        else:
            # Once path is done, signal end and hold the final position for a bit before land
            if not self.scan_ended:
                self.get_logger().info("End of Scan.")
                b = Bool(); b.data = True
                self.scan_end_pub.publish(b)
                self.scan_ended = True
            self.publish_takeoff_setpoint(0.0, 0.0, -self.end_altitude)

        # After 100 more ticks, land the drone
        if self.offboard_arr_counter == (len(self.path) + 100):
            self.figure8_timer.cancel()
            self.land()

        self.offboard_arr_counter += 1

    def vehicle_local_position_callback(self, vehicle_local_position):
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
        self.path = []
        self.hit_figure_8 = False

    def publish_takeoff_setpoint(self, x: float, y: float, z: float):
        """Publish a simple position setpoint for takeoff/hover."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode (heartbeat)."""
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
