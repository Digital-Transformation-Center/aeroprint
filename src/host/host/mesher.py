#!/usr/bin/env python3
"""
mesher.py: ROS node meshing output point cloud.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.1.0"
__status__ = "Beta"

import open3d as o3d
import numpy as np
import trimesh as tm
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String, Bool
import rclpy

class Mesher(Node):
    """Mesher node"""
    def __init__(self) -> None:
        super().__init__("mesher")
        self.get_logger().info("Mesher alive")

        self.dump_directory_sub = self.create_subscription(
             String,
             "/host/out/pcc/dump_directory", 
             self.dump_directory_callback, 
             qos_profile_system_default
        )

        self.export_complete_sub = self.create_subscription(
            Bool, 
            "/host/out/pcpp/export_complete",
            self.export_complete_callback, 
            qos_profile_system_default
        )

        self.file_directory_pub = self.create_publisher(
            String, 
            "/host/out/mesher/file_directory", 
            qos_profile_system_default
        )

        self.directory = ""
        self.pc_file_location = ""

    def dump_directory_callback(self, msg):
        """Callback for dump directory"""
        self.directory = msg.data
        self.pc_file_location = msg.data + "/combined_filtered.pcd"
    
    def export_complete_callback(self, msg):
        """Callback for export complete"""
        self.get_logger().info("Export complete callback: " + str(msg.data))
        if msg.data:
            self.get_logger().info("Meshing...")
            self.save()

    def process(self):
        """FUnction to process mesh"""
        pcd = o3d.io.read_point_cloud(self.pc_file_location)
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            pcd, 0.02)
        mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)
        trimesh_mesh = tm.Trimesh(vertices=mesh.vertices, faces=mesh.triangles)
        trimesh_mesh.fill_holes()
        trimesh_mesh.export("output.stl")

    def save(self):
        """Process, save and publish path of mesh"""
        self.process()
        output_path = String()
        output_path.data = self.directory + "/output.stl"
        self.file_directory_pub.publish(output_path)
        self.get_logger().info("Mesh complete.")

def main(args=None):
    rclpy.init(args=args)
    mesher = Mesher()
    rclpy.spin(mesher)
    mesher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main() 

# pcd_path = "first_test_points/test6/combined_filtered.pcd"
# pcd = o3d.io.read_point_cloud(pcd_path)
# pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
# radii = [0.005, 0.01, 0.02, 0.04]
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
#     pcd, 0.02)
# # Test comment
# # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
# #     pcd, o3d.utility.DoubleVector(radii))
# # mesh.compute_vertex_normals()
# # # mesh = mesh.filter_smooth_simple(number_of_iterations=5)
# mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)
# # mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh).fill_holes().to_legacy()

# trimesh_mesh = tm.Trimesh(vertices=mesh.vertices, faces=mesh.triangles)
# trimesh_mesh.fill_holes()
# trimesh_mesh.export("trimesh_mesh.stl")

# # print('run Poisson surface reconstruction')
# # with o3d.utility.VerbosityContextManager(
# #         o3d.utility.VerbosityLevel.Debug) as cm:
# #     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
# #         pcd, depth=20)
# print(mesh)

# # o3d.visualization.draw_geometries([pcd, mesh])
