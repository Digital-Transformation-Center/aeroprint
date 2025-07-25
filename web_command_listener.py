#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import os

class WebCommandPublisher(Node):
    def __init__(self):
        super().__init__('web_command_publisher')
        self.command_pub = self.create_publisher(String, '/host/gui/in/command', 10)
        self.size_pub = self.create_publisher(String, '/host/gui/out/print_size', 10)
        self.last_sent = {"command": "", "size": ""}

    def run(self):
        while rclpy.ok():
            if os.path.exists("command.json"):
                try:
                    with open("command.json") as f:
                        data = json.load(f)
                        command = data.get("command", "")
                        size = data.get("size", "")

                        if command and command != self.last_sent["command"]:
                            msg = String()
                            msg.data = command
                            self.command_pub.publish(msg)
                            self.get_logger().info(f"🚀 Sent command: {command}")
                            self.last_sent["command"] = command

                        if size and size != self.last_sent["size"]:
                            size_msg = String()
                            size_msg.data = size
                            self.size_pub.publish(size_msg)
                            self.get_logger().info(f"📦 Sent size: {size}")
                            self.last_sent["size"] = size

                except Exception as e:
                    self.get_logger().error(f"Error reading command.json: {e}")

            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = WebCommandPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()