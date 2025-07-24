#!/usr/bin/env python3
"""
pc_post_processor.py: ROS node for combining and filtering individual posed point clouds.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle, Saif Ullah"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.1.0"
__status__ = "Beta"

import open3d as o3d
import os
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Float32, Bool, String, Float32MultiArray
import rclpy

class PCPostProcessor(Node):
    """Node for processing individual point cloud files"""
    def __init__(self) -> None:
        super().__init__("pc_post_processor")

        self.get_logger().info("Point cloud post processor alive")

        # Create subscribers
        # self.radius_sub = self.create_subscription(
        #      Float32,
        #      "/host/gui/out/radius", 
        #      self.radius_callback, 
        #      qos_profile_system_default
        # )
        # self.object_height_sub = self.create_subscription(
        #      Float32,
        #      "/host/gui/out/object_height", 
        #      self.object_height_callback, 
        #      qos_profile_system_default
        # )
        # self.start_height_sub = self.create_subscription(
        #      Float32,
        #      "/host/gui/out/start_height", 
        #      self.start_height_callback, 
        #      qos_profile_system_default
        # )
        self.dump_complete_sub = self.create_subscription(
             Bool,
             "/host/out/pcc/dump_complete", 
             self.dump_complete_callback, 
             qos_profile_system_default
        )
        self.dump_directory_sub = self.create_subscription(
             String,
             "/host/out/pcc/dump_directory", 
             self.dump_directory_callback, 
             qos_profile_system_default
        )
        self.export_complete_pub = self.create_publisher(
            Bool,
            "/host/out/pcpp/export_complete",
            qos_profile_system_default
        )
        self.helix_param_sub = self.create_subscription(
            Float32MultiArray,
            "/helix_params",
            self.helix_params_callback,
            10
        )

        self.pcd_directory_subscriber = self.create_subscription(
            String,
            "/web/pcd_directory",
            self.pcd_directory_callback,
            qos_profile_system_default
        )
        

        # Variables for subscribers to update
        self.radius = 0.0
        self.object_height = 0.0
        self.start_height = 0.0
        self.dump_directory = ""

        # Create publishers
        self.export_complete_pub = self.create_publisher(
            Bool, 
            "/host/out/pcpp/export_complete", 
            qos_profile_system_default
        )
        
        self.combined_pcd_data = []
        # Load all PCD files from the directory
        self.pcd_list = []
        self.combined_pcd = None 

    def pcd_directory_callback(self, msg):
        """Callback for receiving the PCD directory from the web interface."""
        self.get_logger().info(f"Received PCD directory: {msg.data}")
        self.dump_directory = msg.data
        if not os.path.exists(self.dump_directory):
            os.makedirs(self.dump_directory)
            self.get_logger().info(f"Created directory: {self.dump_directory}")

    def helix_params_callback(self, data):
        """Callback for helix parameters."""
        self.get_logger().info(f"Received helix parameters: {data.data}")
        self.radius = data.data[0]
        self.object_height = data.data[1]
        self.start_height = data.data[3]
        # self.flight_params = {
        #     "helix_height": data.data[1],
        #     "helix_radius": data.data[0],
        #     "helix_num_passes": data.data[2],
        #     "helix_start_height": data.data[3],
        #     "helix_rate": 15.0 # Default rate, can be adjusted
        # }

    def dump_complete_callback(self, msg):
        """Callback for dump complete."""
        self.get_logger().info("Dump finished. Combining Pointclouds.")
        if msg.data:
            try:
                self.save()
            except Exception as e:
                self.get_logger().error(f"Error during pcd save: {e}")
            ec = Bool()
            ec.data = True
            self.export_complete_pub.publish(ec)

    def dump_directory_callback(self, msg):
        """Callback for dump directory."""
        self.dump_directory = msg.data

    def down_sample(self):
        """Downsample combined point cloud."""
        self.combined_pcd = self.combined_pcd.voxel_down_sample(voxel_size=0.01)

    def load_pcs(self):
        """Load point clouds from files"""
        for filename in os.listdir(self.dump_directory):
            if filename.endswith(".pcd"):
                pcd_path = os.path.join(self.dump_directory, filename)
                pcd = o3d.io.read_point_cloud(pcd_path)
                self.pcd_list.append(pcd)
        self.combined_pcd = self.pcd_list[0]

    def combine_pcs(self):
        """Combine individual point clouds."""
        for pc in self.pcd_list[1:]:
            self.get_logger().info("Adding pcd to combined")
            self.combined_pcd += pc
            # self.combined_pcd_data = np.concatenate(self.combined_pcd_data, np.asarray(pc.points))

    def filter_pcs(self):
        """Filter combined point cloud."""

        points = np.asarray(self.combined_pcd.points)
        self.get_logger().info(f"Number of points before filtering: {len(points)}")
        points = self.confine_to_circle(points, self.radius)
        self.get_logger().info(f"Number of points after confining to circle: {len(points)}")
        max_z = self.start_height + self.object_height + 0.3
        points = self.confine_to_z(points, self.start_height, max_z)
        self.combined_pcd = o3d.geometry.PointCloud()
        self.combined_pcd.points = o3d.utility.Vector3dVector(points)
        self.combined_pcd, ind = self.combined_pcd.remove_statistical_outlier(nb_neighbors=10,
                                                    std_ratio=2.2)

    def confine_to_circle(self, points, r):
        """Confine points to a circle."""
        self.get_logger().info(f"Confine to circle with radius: {r}")
        center_x = r
        center_y = 0.0
        distances = np.sqrt((points[:, 0] - center_x) ** 2 + (points[:, 1] - center_y) ** 2)
        mask = distances <= r
        return points[mask]

    def confine_to_z(self, points, min_z, max_z):
        """Confine points within certain Z boundaries."""
        self.get_logger().info(f"Confine to Z, bottom: {min_z}, top: {max_z}")
        z_height = points[:, 2]
        mask = points[:, 2] < -min_z # and points[:, 2] > -max_z
        # mask = z_height < -min_z and z_height > -max_z
        return points[mask]
    
    def save(self):
        """Process points and save to a file."""
        self.load_pcs()
        self.combine_pcs()
        self.down_sample()
        self.filter_pcs()
        self.get_logger().info("Writing file to " + self.dump_directory + "/combined_filtered.pcd")
        o3d.io.write_point_cloud(self.dump_directory + "/combined_filtered.pcd", self.combined_pcd)
        self.pcd_list = []

def main(args=None):
    rclpy.init(args=args)
    pc_post_processor = PCPostProcessor()
    rclpy.spin(pc_post_processor)
    pc_post_processor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main() 
