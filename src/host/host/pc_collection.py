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
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node
import numpy as np
import open3d as o3d

class PCNode(Node):
   """Class for dumping ROS PointCloud2 messages to files"""
   def __init__(self) -> None:
      # Create the node and subscriber
      super().__init__("point_cloud_handler_node")
      self.get_logger().info("Point cloud collector alive")

      self.pc2_sub = self.create_subscription(
         PointCloud2, 
         "/starling/out/relative_posed_pc", 
         self.pc2_callback, 
         qos_profile_sensor_data
      )

      self.scan_start_sub = self.create_subscription(
         Bool, 
         "/fcu/out/start_scan",
         self.scan_start_callback,
         qos_profile_system_default
      )

      self.scan_end_sub = self.create_subscription(
         Bool, 
         "/fcu/out/end_scan",
         self.scan_end_callback,
         qos_profile_system_default
      )

      self.pcd_directory_subscriber = self.create_subscription(
         String,
         "/web/pcd_directory",
         self.pcd_directory_callback,
         qos_profile_system_default
      )

      self.dump_complete_pub = self.create_publisher(
         Bool, 
         "/host/out/pcc/dump_complete", 
         qos_profile_system_default
      )

      self.scan_start = False
      self.scan_end = False

      self.dump_dir = ""
      self.pcd_dir = ""
      self.pc_interval = 2.0 # Point cloud interval in seconds
      # Keep a timestamp to limit scan frequency
      self.last_pc = self.get_clock().now().nanoseconds

   def scan_start_callback(self, msg):
      """Start scanning"""
      self.get_logger().info("Scan start received.")
      self.scan_start = msg.data
      self.scan_end = not self.scan_start

   def scan_end_callback(self, msg):
      """Stop scanning"""
      self.scan_end = msg.data
      self.scan_start = not self.scan_end
      #   dc = Bool()
      #   dc.data = True
      self.dump_complete_pub.publish(msg)

   def pcd_directory_callback(self, msg):
      """Callback for receiving the PCD directory from the web interface."""
      self.get_logger().info(f"Received PCD directory: {msg.data}")
      self.pcd_dir = msg.data

   def pc2_callback(self, data):
      """Dump point clouds if scan started"""
      if self.scan_start and not self.scan_end:
         now = self.get_clock().now().nanoseconds
         if now - self.last_pc >= self.pc_interval * 1e9:
            try:
               self.get_logger().info("Saving pointcloud data...")
               # Convert ROS PointCloud2 to Open 3D point cloud
               points = np.frombuffer(data.data, dtype=np.float32).reshape(-1, 3)
               self.get_logger().info("Converted PointCloud2 to Open3D format.")
               o3dpc = o3d.geometry.PointCloud()
               o3dpc.points = o3d.utility.Vector3dVector(points)
               self.get_logger().info("Created Open3D point cloud.")
               num_points = len(points)
               self.get_logger().info("Pointcloud with points: " + str(num_points))

               # Write point clouds to files
               o3d.io.write_point_cloud(self.pcd_dir + "/pointcloud" + str(data.header.stamp.nanosec) + ".pcd", o3dpc)
               self.last_pc = now # Update time
            except Exception as e:
               self.get_logger().warn("Error in pcd processing...")
               self.get_logger().info(e)


def main(args=None) -> None:
    rclpy.init(args=args)
    pc_node = PCNode()
    rclpy.spin(pc_node)
    pc_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
