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
import trimesh as tm # Import trimesh
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

    def voxel_repair(self, mesh, pitch=None):
        """
        Reconstructs a watertight mesh by voxelizing the input mesh and
        extracting the surface with marching cubes.
        """
        try:
            self.get_logger().info("Attempting voxel-based repair...")

            if pitch is None:
                bounds = mesh.bounds
                scale = bounds[1] - bounds[0]
                pitch = min(scale) / 100  # dynamic pitch based on model size

            voxelized = mesh.voxelized(pitch)
            filled = voxelized.fill()

            watertight_mesh = filled.marching_cubes

            # Clean and return
            watertight_mesh.update_faces(watertight_mesh.unique_faces())
            watertight_mesh.remove_unreferenced_vertices()
            watertight_mesh.merge_vertices()

            self.get_logger().info("Voxel repair succeeded.")
            return watertight_mesh

        except Exception as e:
            self.get_logger().error(f"Voxel repair failed: {e}")
            return None

    def check_and_fix_watertightness(self, mesh):
        """
        Loads an STL mesh, checks and attempts to fix watertightness,
        and saves the repaired mesh if possible.
        """
        if mesh.is_watertight:
            self.get_logger().info(f"Initial watertight: {mesh.is_watertight}")
        else:
            self.get_logger().warn(f"Initial watertight: {mesh.is_watertight}")

        # Initial cleaning
        mesh.update_faces(mesh.unique_faces())
        mesh.remove_unreferenced_vertices()
        mesh.merge_vertices()
        tm.repair.fill_holes(mesh)

        self.get_logger().info(f"Watertight after basic cleanup: {mesh.is_watertight}")

        # Try voxel repair if still not watertight
        if not mesh.is_watertight:
            voxel_mesh = self.voxel_repair(mesh)
            if voxel_mesh and voxel_mesh.is_watertight:
                mesh = voxel_mesh
            else:
                self.get_logger().error("Mesh remains non-watertight after all attempts.")
                return

        # Final cleanup
        mesh.remove_degenerate_faces()
        mesh.fix_normals()
        mesh.remove_infinite_values()

        return mesh  
    
    def process(self):
        """
        Processes a point cloud file to generate a smoothed and filtered 3D mesh.
        Steps:

        1. Reads the point cloud from the specified file location.
        2. Estimates normals for the point cloud.
        3. Creates a triangle mesh from the point cloud using alpha shapes.
        4. Smooths the mesh using Laplacian smoothing.
        5. Clusters connected triangles and removes small clusters.
        6. **Performs mesh repair using trimesh.**
        7. Computes vertex normals for the final mesh.
        8. Exports the final mesh to an STL file.

        Attributes:
            self.pc_file_location (str): The file location of the point cloud.
            self.directory (str): The directory where the output STL file will be saved.
        Returns:
        None
        """

        self.get_logger().info(f"Reading point cloud from: {self.pc_file_location}")
        pcd = o3d.io.read_point_cloud(self.pc_file_location)
        if not pcd.has_points():
            self.get_logger().error("Input PCD file has no points. Aborting meshing.")
            return

        self.get_logger().info("Estimating normals...")
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

        self.get_logger().info("Creating mesh from point cloud using alpha shape...")
        # Experiment with alpha value (e.g., 0.04). Too small may leave holes, too large may include noise.
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 0.04)
        if not mesh.has_triangles():
            self.get_logger().error("Alpha shape did not create any triangles. Try adjusting alpha value or check point cloud quality.")
            return

        self.get_logger().info("Smoothing mesh using Laplacian smoothing...")
        mesh = mesh.filter_smooth_laplacian(number_of_iterations=15)

        self.get_logger().info("Clustering connected triangles and removing small clusters...")
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            triangle_clusters, cluster_n_triangles, cluster_area = (
                mesh.cluster_connected_triangles())
        triangle_clusters = np.asarray(triangle_clusters)
        cluster_n_triangles = np.asarray(cluster_n_triangles)
        cluster_area = np.asarray(cluster_area)

        # Remove all clusters except the largest one (assuming the object is the largest cluster)
        if len(cluster_n_triangles) > 0:
            largest_cluster_idx = np.argmax(cluster_n_triangles)
            triangles_to_remove_mask = (triangle_clusters != largest_cluster_idx)
            mesh.remove_triangles_by_mask(triangles_to_remove_mask)
            self.get_logger().info(f"Removed {np.sum(triangles_to_remove_mask)} triangles from smaller clusters.")
        else:
            self.get_logger().warning("No triangle clusters found after initial meshing. This might indicate an issue with the alpha shape or point cloud.")

        # --- Mesh Repair and Validation with Trimesh and Open3D ---
        self.get_logger().info("Converting Open3D mesh to Trimesh for robust repair...")
        # Convert Open3D mesh to Trimesh
        tm_mesh = tm.Trimesh(vertices=np.asarray(mesh.vertices),
                             faces=np.asarray(mesh.triangles),
                             vertex_normals=np.asarray(mesh.vertex_normals))

        self.get_logger().info("Performing mesh repair using Trimesh...")

        
        repaired_mesh = self.check_and_fix_watertightness(tm_mesh)

        self.get_logger().info("Exporting mesh")
        repaired_mesh.export(self.directory + "-mesh-output.stl")
        # Use trimesh's export to STL, as it's often more robust for repaired meshes
        # tm_mesh.export(self.directory + "-mesh-output.stl")
        # Or if you want to stick with Open3D for final export, use the converted mesh_final
        # o3d.io.write_triangle_mesh(self.directory + "-mesh-output.stl", mesh_final)
        self.get_logger().info(f"Mesh saved to: {self.directory}-mesh-output.stl")

    def save(self):
        """
        Processes the mesh and saves the output to a file.
        This method processes the mesh, constructs the output file path,
        logs the output path, publishes the file directory, and logs the
        completion of the mesh processing.
        """

        self.process()
        output_path = String()
        output_path.data = self.directory + "-mesh-output.stl"
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