import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry


class StarlingDataNode(Node):
    """Node for reading some starling data as a test."""

    def __init__(self) -> None:
        super().__init__("starling_data_node")

        self.get_logger().info("Starling data test node alive!")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vehicle_status_subscriber = self.create_subscription(
            Odometry, "/qvio", self.vehicle_odo_callback, qos_profile
        )

    def vehicle_odo_callback(self, vehicle_odometry):
        """Callback function for vehicle_status topic subscriber."""
        print("HELLO")


def main(args=None) -> None:
    rclpy.init(args=args)
    starling_data_node = StarlingDataNode()
    rclpy.spin(starling_data_node)
    starling_data_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
