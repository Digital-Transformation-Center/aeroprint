#!/usr/bin/env python3
"""
mesher.py: ROS node meshing output point cloud.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle, Saif Ullah"
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
        """Function to process mesh"""
        pcd = o3d.io.read_point_cloud(self.pc_file_location)
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            pcd, 0.04)
        # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)
        mesh = mesh.filter_smooth_laplacian(number_of_iterations=15)
        # print("Cluster connected triangles")
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            triangle_clusters, cluster_n_triangles, cluster_area = (
                mesh.cluster_connected_triangles())
        triangle_clusters = np.asarray(triangle_clusters)
        cluster_n_triangles = np.asarray(cluster_n_triangles)
        cluster_area = np.asarray(cluster_area)
        triangles_to_remove = cluster_n_triangles[triangle_clusters] < max(cluster_n_triangles[triangle_clusters])
        mesh.remove_triangles_by_mask(triangles_to_remove)
        # mesh.compute_vertex_normals()

        # edge_manifold = mesh.is_edge_manifold(allow_boundary_edges=True)
        # edge_manifold_boundary = mesh.is_edge_manifold(allow_boundary_edges=False)
        # vertex_manifold = mesh.is_vertex_manifold()
        # self_intersecting = mesh.is_self_intersecting()
        # watertight = mesh.is_watertight()
        # orientable = mesh.is_orientable()

        # print(f"edge_manifold: {edge_manifold}")
        # print(f"edge_manifold_boundary: {edge_manifold_boundary}")
        # print(f"vertex_manifold: {vertex_manifold}")
        # print(f"self_intersecting: {self_intersecting}")
        # print(f"watertight: {watertight}")
        # print(f"orientable: {orientable}")


        # Convert the voxel grid to a point cloud
        self.get_logger().info("Exporting mesh")
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(self.directory + "-output.stl", mesh)
    def save(self):
        """Process, save and publish path of mesh"""
        self.process()
        output_path = String()
        output_path.data = self.directory + "-output.stl"
        self.get_logger().info("Output mesh: " + output_path.data)
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
