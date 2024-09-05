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
from host.file_manager import FileManager

class PCNode(Node):
  """Class for dumping ROS PointCloud2 messages to files"""
  def __init__(self) -> None:
    self.file_manager = FileManager()
    # Create the node and subscriber
    super().__init__("point_cloud_handler_node")
    self.get_logger().info("Point cloud collector alive")

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

    self.scan_dataset_sub = self.create_subscription(
       String, 
       "/host/gui/out/scan_ds", 
       self.scan_dataset_callback, 
       qos_profile_system_default
    )
    
    self.dump_directory_pub = self.create_publisher(
       String, 
       "/host/out/pcc/dump_directory", 
       qos_profile_system_default
    )

    self.dump_complete_pub = self.create_publisher(
       Bool, 
       "/host/out/pcc/dump_complete", 
       qos_profile_system_default
    )

    path = os.path.dirname(os.path.abspath(__file__))  
   # Split the path into components
    path_parts = path.split(os.sep)

   # Find the index of 'aeroprint' in the path
    try:
       aeroprint_index = path_parts.index("aeroprint")
    except ValueError:
       print("Error: 'aeroprint' not found in the path")
    else:
       # Construct the truncated path
       truncated_path = os.sep.join(path_parts[:aeroprint_index + 1])

    print("Truncated path:", truncated_path)
    self.output_path = os.path.join(truncated_path, 'scans/')


    self.scan_start = False
    self.scan_end = False

    self.dump_dir = self.output_path # Dump directory
    self.pc_interval = 2.0 # Point cloud interval in seconds
    # Keep a timestamp to limit scan frequency
    self.last_pc = self.get_clock().now().nanoseconds

  def scan_start_callback(self, msg):
     """Start scanning"""
     self.get_logger().info("Scan start received.")
     try:
      self.get_logger().info("Attempting to create " + self.dump_dir)
      os.mkdir(self.dump_dir)
      self.get_logger().info("Output Directory: " + str(self.dump_dir))
     except OSError as e:
        self.get_logger().info(f"Failed to create directory: {e}")
     self.get_logger().info("Dump directory created.")
     self.scan_start = msg.data
     self.scan_end = not self.scan_start

  def scan_end_callback(self, msg):
     """Stop scanning"""
     self.scan_end = msg.data
   #   dc = Bool()
   #   dc.data = True
     self.dump_complete_pub.publish(msg)


  def scan_title_callback(self, msg):
     """Make file directory from title, save and publish to ros."""
     raw_title = msg.data
   #   file_directory = raw_title.replace("/", "").replace("\\", "").replace(" ", "-").lower()
   #   self.get_logger().info("File directory: " + file_directory)
   #   self.dump_dir = os.path.join(self.output_path, file_directory)
   
     self.file_manager.set_class(raw_title)
     
     self.dump_dir = self.file_manager.get_pointclouds_path()
     self.get_logger().info("Changing dump directory to: " + str(self.dump_dir))
     dump_dir_ros_msg = String()
     dump_dir_ros_msg.data = self.dump_dir
     self.dump_directory_pub.publish(dump_dir_ros_msg)

  def scan_dataset_callback(self, msg):
     self.file_manager.set_dataset(msg.data)

  def pc2_callback(self, data):
    """Dump point clouds if scan started"""
    if not os.path.exists(self.dump_dir):
        os.makedirs(self.dump_dir)
    if self.scan_start and not self.scan_end:
      now = self.get_clock().now().nanoseconds
      if now - self.last_pc >= self.pc_interval * 1e9:
          print(data.header.stamp.nanosec) # For debugging
          # Convert ROS PointCloud2 to Open 3D point cloud
          points = np.frombuffer(data.data, dtype=np.float32).reshape(-1, 3)
          o3dpc = o3d.geometry.PointCloud()
          o3dpc.points = o3d.utility.Vector3dVector(points)
          num_points = len(points)
          self.get_logger().info("Pointcloud with points: " + str(num_points))

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
