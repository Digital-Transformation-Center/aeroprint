import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from voxl_reset_qvio import VOXLQVIOController

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
        super().__init__("offboard_figure8_node")

        self.get_logger().info("Offboard Figure 8 Node Alive!")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create publishers
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

        self.voxl_reset = VOXLQVIOController()

        self.rate = 20
        self.radius = 0.9
        self.cycle_s = 18
        
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
        self.min_altitude = -0.60
        self.altitude = -0.60
        self.init_path()
        self.altitude = -1.10
        self.init_path()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def init_path(self):

        dt = 1.0 / self.rate
        dadt = (2.0 * math.pi) / self.cycle_s
        r = self.radius

        for i in range(self.steps):
            msg = TrajectorySetpoint()

            a = (-math.pi) + i * (2.0 * math.pi / self.steps)
            

            msg.position = [r + r * math.cos(a), r * math.sin(a), self.altitude]
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
            self.publish_takeoff_setpoint(0.0, 0.0, self.min_altitude)
        else:
            if not self.hit_figure_8:
                self.get_logger().info("Flying Circle Now")
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
        # self.hit_figure_8 = False

    def offboard_move_callback(self):
        if self.offboard_arr_counter < len(self.path):
            self.trajectory_setpoint_publisher.publish(
                self.path[self.offboard_arr_counter]
            )

        if self.offboard_arr_counter >= len(self.path):
            self.publish_takeoff_setpoint(0.0, 0.0, self.min_altitude)

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


def main(args=None) -> None:
    rclpy.init(args=args)
    offboard_figure8_node = OffboardFigure8Node()
    rclpy.spin(offboard_figure8_node)
    offboard_figure8_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
