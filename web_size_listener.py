import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import json
import time

class WebSizePublisher(Node):
    def __init__(self):
        super().__init__('web_size_publisher')
        self.publisher = self.create_publisher(Float32, '/host/gui/out/radius', 10)
        self.last_value = 0.0

    def run(self):
        while rclpy.ok():
            try:
                with open("command.json") as f:
                    data = json.load(f)
                    size = data.get("size", "MED")
                    size_map = {"SM": 1.0, "MED": 2.0, "LG": 3.5}
                    radius = size_map.get(size, 2.0)

                    if radius != self.last_value:
                        msg = Float32()
                        msg.data = radius
                        self.publisher.publish(msg)
                        self.get_logger().info(f"📡 Sent radius: {radius}")
                        self.last_value = radius

            except:
                pass
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = WebSizePublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
# This code is a ROS 2 node that reads the size from a JSON file and publishes it as a Float32 message.
# It runs in a loop, checking the file every second, and publishes the radius if it has changed since the last publication.
# The size is mapped to a radius value, and the node logs the sent radius to the console.
