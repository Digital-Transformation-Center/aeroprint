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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, BatteryStatus


class HeartbeatNode(Node):
    """Node for sending a heartbeat signal to PX4 to maintain offboard mode."""

    def __init__(self) -> None:
        super().__init__("heartbeat_node")

        self.get_logger().info("Heartbeat Node Alive!")

        # Configure QoS profile for publishing
        # Publishers to PX4 typically use BEST_EFFORT
        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Configure QoS profile for subscribing
        # Subscribers from PX4 can use BEST_EFFORT or RELIABLE depending on the topic
        # Increasing depth can help with history buffer size issues like the one encountered.
        # Explicitly set BEST_EFFORT to match PX4's typical publisher settings for status topics.
        qos_profile_subscriber = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # To allow late subscribers to get the last message
            history=HistoryPolicy.KEEP_LAST,
            depth=5,  # Increased depth slightly to avoid previous payload size issues
        )

        # Create publisher for OffboardControlMode
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile_publisher
        )

        # Create subscriber for BatteryStatus
        self.battery_status_subscriber = self.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status",
            self.battery_status_callback,
            qos_profile_subscriber, # Use the subscriber QoS profile
        )

        # Timer to publish heartbeat signal periodically (e.g., every 0.1 seconds = 10Hz)
        # PX4 typically requires OffboardControlMode messages at >2Hz.
        self.heartbeat_timer_period = 0.1  # seconds
        self.timer = self.create_timer(
            self.heartbeat_timer_period, self.publish_heartbeat_signal
        )

        self.get_logger().info(
            f"Heartbeat signal will be published every {self.heartbeat_timer_period} seconds."
        )

    def publish_heartbeat_signal(self) -> None:
        """Publish the offboard control mode signal (heartbeat)."""
        msg = OffboardControlMode()
        msg.position = True  # We intend to control position
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # PX4 timestamp is in microseconds

        self.offboard_control_mode_publisher.publish(msg)
        # self.get_logger().info("Heartbeat signal sent.", throttle_duration_sec=1.0) # Optional: for debugging

    def battery_status_callback(self, msg: BatteryStatus) -> None:
        """Callback function for battery_status topic subscriber."""
        # The 'remaining' field is a float from 0.0 (empty) to 1.0 (full)
        # NAN if unknown.
        if not msg.remaining != msg.remaining: # Check for NAN
            battery_percentage = msg.remaining * 100
            self.get_logger().info(f"Battery Remaining: {battery_percentage:.1f}%", throttle_duration_sec=5.0)

    def destroy_node(self):
        self.get_logger().info("Shutting down Heartbeat Node.")
        super().destroy_node()


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