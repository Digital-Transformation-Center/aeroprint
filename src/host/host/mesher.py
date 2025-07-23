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
    """
    Mesher class responsible for processing point cloud data into a mesh and exporting it.
    """

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
        # self.log_directory_subscriber = self.create_subscription(
        #     String,
        #     "/web/log_directory",
        #     self.log_directory_callback,
        #     qos_profile_system_default
        # )
        self.pcd_directory_subscriber = self.create_subscription(
         String,
         "/web/pcd_directory",
         self.pcd_directory_callback,
         qos_profile_system_default
      )

        self.directory = ""
        self.pc_file_location = ""

    def pcd_directory_callback(self, msg):
        self.directory = msg.data
        self.pc_file_location = msg.data + "/combined_filtered.pcd"

    def dump_directory_callback(self, msg):
        """
        Callback function to handle directory dump messages.
        This function sets the directory and point cloud file location based on the incoming message data.
        Args:
            msg (Message): The message containing the directory path data.
        """

        self.directory = msg.data
        self.pc_file_location = msg.data + "/combined_filtered.pcd"
    
    def export_complete_callback(self, msg):
        """
        Callback function that is triggered when the export process is complete.
        Args:
            msg (Message): The message object containing the export status.
        Logs:
            - "Export complete callback: <msg.data>" to indicate the callback has been triggered.
            - "Meshing..." if the export was successful.
        Actions:
            - Calls the save() method if the export was successful.
        """

        self.get_logger().info("Export complete callback: " + str(msg.data))
        if msg.data:
            self.get_logger().info("Meshing...")
            try:
                self.save()
            except Exception as e:
                self.get_logger().error(f"Unable to complete mesh: {e}")

    def process(self):
        """
        Processes a point cloud file to generate a smoothed and filtered 3D mesh.
        Steps:

        1. Reads the point cloud from the specified file location.
        2. Estimates normals for the point cloud.
        3. Creates a triangle mesh from the point cloud using alpha shapes.
        4. Smooths the mesh using Laplacian smoothing.
        5. Clusters connected triangles and removes small clusters.
        6. Computes vertex normals for the final mesh.
        7. Exports the final mesh to an STL file.
        
        Attributes:
            self.pc_file_location (str): The file location of the point cloud.
            self.directory (str): The directory where the output STL file will be saved.
        Returns:
        None
        """

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
        """
        Processes the mesh and saves the output to a file.
        This method processes the mesh, constructs the output file path,
        logs the output path, publishes the file directory, and logs the
        completion of the mesh processing.
        """

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
