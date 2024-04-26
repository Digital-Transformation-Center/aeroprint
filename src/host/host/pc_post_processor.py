#!/usr/bin/env python3
"""
pc_post_processor.py: ROS node for combining and filtering individual posed point clouds.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.1.0"
__status__ = "Beta"

import open3d as o3d
import os
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Float32, Bool, String
import rclpy

class PCPostProcessor(Node):
    """Node for processing individual point cloud files"""
    def __init__(self) -> None:
        super().__init__("pc_post_processor")

        self.get_logger().info("Point cloud post processor alive")

        # Create subscribers
        self.radius_sub = self.create_subscription(
             Float32,
             "/host/gui/out/flight_radius", 
             self.radius_callback, 
             qos_profile_system_default
        )
        self.object_height_sub = self.create_subscription(
             Float32,
             "/host/gui/out/object_height", 
             self.object_height_callback, 
             qos_profile_system_default
        )
        self.start_height_sub = self.create_subscription(
             Float32,
             "/host/gui/out/start_height", 
             self.start_height_callback, 
             qos_profile_system_default
        )
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
        
    
    def radius_callback(self, msg):
        """Callback for radius."""
        self.radius = msg.data

    def object_height_callback(self, msg):
        """Callback for object height."""
        self.object_height = msg.data

    def start_height_callback(self, msg):
        """Callback for starting height."""
        self.start_height = msg.data

    def dump_complete_callback(self, msg):
        """Callback for dump complete."""
        if msg.data:
            self.save()

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
            print("adding pcd...")
            self.combined_pcd += pc
            # self.combined_pcd_data = np.concatenate(self.combined_pcd_data, np.asarray(pc.points))

    def filter_pcs(self):
        """Filter combined point cloud."""
        points = np.asarray(self.combined_pcd.points)
        points = self.confine_to_circle(points, self.radius)
        max_z = self.start_height + self.object_height + 0.5
        points = self.confine_to_z(points, self.start_height, max_z)
        self.combined_pcd = o3d.geometry.PointCloud()
        self.combined_pcd.points = o3d.utility.Vector3dVector(points)
        self.combined_pcd, ind = self.combined_pcd.remove_statistical_outlier(nb_neighbors=10,
                                                    std_ratio=2.2)


    def confine_to_circle(self, points, r):
        """Confine points to a circle."""
        center_x = r
        center_y = 0.0
        distances = np.sqrt((points[:, 0] - center_x) ** 2 + (points[:, 1] - center_y) ** 2)
        mask = distances <= r
        return points[mask]

    def confine_to_z(self, points, min_z, max_z):
        """COnfine points within certain Z boundaries."""
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
        o3d.io.write_point_cloud(self.dump_directory + "/combined_filtered.pcd", self.combined_pcd)

def main(args=None):
    rclpy.init(args=args)
    pc_post_processor = PCPostProcessor()
    rclpy.spin(pc_post_processor)
    pc_post_processor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main() 
