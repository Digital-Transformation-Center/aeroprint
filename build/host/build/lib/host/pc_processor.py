
import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2

class PCProcessingNode(Node):
    def __init__(self) -> None:
        super().__init__('pc_processing_node')

        self.get_logger().info("Point cloud processing node alive!")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        try:
            self.pc_subscriber = self.create_subscription(PointCloud2, '/voa_pc_out', self.pc_message_callback, qos_profile)
            self.get_logger().info("Started subscriber.")
        except Exception as e:
            self.get_logger().error(e)
        # self.pc_publisher = self.create_publisher(PointCloud2, '/my_pc_out', qos_profile)

    def pc_message_callback(self, msg):
        print("HI")
        # self.pc_publisher.publish(msg)


def main(args=None) -> None:
    print("Started subscriber.")
    rclpy.init(args=args)
    pc_processing_node = PCProcessingNode()
    rclpy.spin(pc_processing_node)
    pc_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)