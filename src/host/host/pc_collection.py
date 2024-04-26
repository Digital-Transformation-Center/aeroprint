#!/usr/bin/env python3
"""
pc_collection.py: ROS node for collecting and saving point clouds.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.1.0"
__status__ = "Beta"

import rclpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
import open3d as o3d
import numpy as np
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node
import numpy as np
import open3d as o3d
import os

class PCNode(Node):
  """Class for dumping ROS PointCloud2 messages to files"""
  def __init__(self) -> None:
    self.get_logger().info("Point cloud collector alive")

    # Create the node and subscriber
    super().__init__("point_cloud_handler_node")
    self.pc2_sub = self.create_subscription(
      PointCloud2, 
      "/starling/out/posed_pc", 
      self.pc2_callback, 
      qos_profile_sensor_data
    )

    self.scan_start_sub = self.create_subscription(
      Bool, 
      "/starling/out/fc/scan_start",
      self.scan_start_callback,
      qos_profile_system_default
    )

    self.scan_end_sub = self.create_subscription(
      Bool, 
      "/starling/out/fc/scan_end",
      self.scan_end_callback,
      qos_profile_system_default
    )

    self.scan_title_sub = self.create_subscription(
       String, 
       "/host/gui/out/scan_title", 
       self.scan_title_callback, 
       qos_profile_system_default
    )
    
    self.dump_directory_pub = self.create_publisher(
       String, 
       "/host/out/pcc/dump_directory", 
       qos_profile_system_default
    )

    

    self.scan_start = False
    self.scan_end = True

    self.dump_dir = "" # Dump directory
    self.pc_interval = 1.0 # Point cloud interval in seconds
    # Keep a timestamp to limit scan frequency
    self.last_pc = self.get_clock().now().nanoseconds

  def scan_start_callback(self, msg):
     """Start scanning"""
     self.scan_start = msg.data

  def scan_end_callback(self, msg):
     """Stop scanning"""
     self.scan_end = msg.data

  def scan_title_callback(self, msg):
     """Make file directory from title, save and publish to ros."""
     raw_title = msg.data
     file_directory = raw_title.replace("/", "").replace("\\", "").replace(" ", "").lower()
     self.dump_dir = file_directory
     os.mkdir(self.dump_dir)
     dump_dir_ros_msg = String()
     dump_dir_ros_msg.data = self.dump_dir
     self.dump_directory_pub.publish(dump_dir_ros_msg)

  def pc2_callback(self, data):
    """Dump point clouds if scan started"""
    if self.scan_start and not self.scan_end:
      now = self.get_clock().now().nanoseconds
      if now - self.last_pc >= self.pc_interval * 1e9:
          print(data.header.stamp.nanosec) # For debugging
          # Convert ROS PointCloud2 to Open 3D point cloud
          points = np.frombuffer(data.data, dtype=np.float32).reshape(-1, 3)
          o3dpc = o3d.geometry.PointCloud()
          o3dpc.points = o3d.utility.Vector3dVector(points)

          # Write point clouds to files
          o3d.io.write_point_cloud(self.dump_dir + "/pointcloud" + str(data.header.stamp.nanosec) + ".pcd", o3dpc)
          self.last_pc = now # Update time

def main(args=None) -> None:
    rclpy.init(args=args)
    pc_node = PCNode()
    rclpy.spin(pc_node)
    pc_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
    
