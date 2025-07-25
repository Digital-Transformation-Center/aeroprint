import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json

class StageListener(Node):
    def __init__(self):
        super().__init__('stage_listener')
        self.get_logger().info("🛰️ Stage listener online")

        # Subscribing to multiple stage updates
        self.create_subscription(Bool, "/host/out/pcc/dump_complete", self.handle_scanning, 10)
        self.create_subscription(Bool, "/host/out/pcpp/export_complete", self.handle_processing, 10)
        self.create_subscription(String, "/host/out/mesher/file_directory", self.handle_meshing, 10)

    def save_stage(self, stage):
        # Read existing status if file exists
        try:
            with open("status.json", "r") as f:
                status = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            status = {}
        status["stage"] = stage
        with open("status.json", "w") as f:
            json.dump(status, f)
        self.get_logger().info(f"📁 Updated stage: {stage}")

    def handle_scanning(self, msg):
        if msg.data:
            self.save_stage("Scanning")

    def handle_processing(self, msg):
        if msg.data:
            self.save_stage("Processing")

    def handle_meshing(self, msg):
        self.save_stage("Meshing")

def main(args=None):
    rclpy.init(args=args)
    listener = StageListener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
