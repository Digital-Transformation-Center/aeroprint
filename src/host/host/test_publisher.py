#!/usr/bin/env python3
"""
test_publisher.py: ROS node for testing.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.1.0"
__status__ = "Beta"

from rclpy.node import Node
import rclpy
from std_msgs.msg import Bool, String, Float32
from rclpy.qos import qos_profile_system_default
# from voxl_reset_qvio import VOXLQVIOController


class TestPublisher(Node):
    def __init__(self) -> None:
        super().__init__("test_publisher_node")

        self.ready_pub = self.create_publisher(
            Bool, 
            "/host/gui/out/ready", 
            qos_profile_system_default
        )
        self.radius_pub = self.create_publisher(
            Float32, 
            "/host/gui/out/radius", 
            qos_profile_system_default
        )
        self.object_height_pub = self.create_publisher(
            Float32, 
            "/host/gui/out/object_height",
            qos_profile_system_default
        )
        self.start_height_pub = self.create_publisher(
            Float32, 
            "/host/gui/out/start_height", 
            qos_profile_system_default
        )
        self.scan_title_pub = self.create_publisher(
            String, 
            "/host/gui/out/scan_title", 
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
        self.file_directory_pub = self.create_publisher(
            String, 
            "/host/out/mesher/file_directory", 
            qos_profile_system_default
        )
        self.dump_directory_pub = self.create_publisher(
             String,
             "/host/out/pcc/dump_directory", 
             qos_profile_system_default
        )
        self.export_complete_pub = self.create_publisher(
            Bool, 
            "/host/out/pcpp/export_complete",
            qos_profile_system_default
        )
        # self.qvio_reset = VOXLQVIOController()
    
    def run(self):
        self.ready_pub.publish(self.create_bool(False))
        self.radius_pub.publish(self.create_float32(0.75))
        self.object_height_pub.publish(self.create_float32(0.6))
        self.start_height_pub.publish(self.create_float32(0.5))
        self.scan_title_pub.publish(self.create_string("Test Data"))
        self.ready_pub.publish(self.create_bool(True))

    def start_flight(self):
        self.qvio_reset.reset()

    def scan_param(self, title):
        self.scan_title_pub.publish(self.create_string(title))

    def start_scan(self):
        self.scan_start_pub.publish(self.create_bool(True))

    def end_scan(self):
        self.scan_end_pub.publish(self.create_bool(True))

    def gcode(self, path):
        self.file_directory_pub.publish(self.create_string(path))

    def mesh(self, dir):
        self.dump_directory_pub.publish(self.create_string(dir))
        self.export_complete_pub.publish(self.create_bool(True))

    def create_float32(self, val):
        res = Float32()
        res.data = val
        return res
    
    def create_bool(self, val):
        res = Bool()
        res.data = val
        return res

    def create_string(self, val):
        res = String()
        res.data = val
        return res

def main(args=None) -> None:
    rclpy.init(args=args)
    tp = TestPublisher()
    go = True
    while go:
        _ = input("Enter Command, h for help:")
        if _ == "ready": tp.run()
        elif _ == "start-flight": tp.start_flight()
        elif _ == "start-scan": 
            title = input("Enter scan title: ")
            tp.scan_param(title)
            q = input("Press enter to start scan")
            tp.start_scan()
        elif _ == "end-scan": tp.end_scan()
        elif _ == "gcode": 
            output_path = input("Enter stl path: ")
            tp.gcode(output_path)
        elif _ == "q": go = False
        elif _ == "mesh":
            dir = input("Enter folder with pcd: ")
            tp.mesh(dir)


if __name__ == "__main__":
    main()
    
        

