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

        self.get_logger().info("Checking mesh properties before repair:")
        self.get_logger().info(f"Is watertight (trimesh): {tm_mesh.is_watertight}")
        self.get_logger().info(f"Is manifold (trimesh): {tm_mesh.is_manifold}")
        self.get_logger().info(f"Has faces (trimesh): {tm_mesh.has_faces}")
        self.get_logger().info(f"Is self-intersecting (trimesh): {tm_mesh.is_self_intersecting}")

        if not tm_mesh.is_watertight:
            self.get_logger().warning("Mesh is not watertight. Attempting repair with Trimesh.")
            # Fill holes and make it watertight
            tm_mesh.fill_holes()
            # If still not watertight, try `repair.make_watertight` (more aggressive)
            if not tm_mesh.is_watertight:
                self.get_logger().info("Mesh still not watertight after fill_holes. Trying `repair.make_watertight`.")
                # This function tries to make the mesh watertight by attempting to fill holes and ensuring consistent orientation
                tm_mesh = tm.repair.make_watertight(tm_mesh)

        if not tm_mesh.is_manifold:
            self.get_logger().warning("Mesh is not manifold. Attempting to repair non-manifold edges with Trimesh.")
            # Remove non-manifold edges. This can sometimes remove valid geometry if the initial mesh is very broken.
            tm_mesh.remove_nonmanifold_edges()
            # Also try to fix non-manifold vertices
            tm_mesh.remove_nonmanifold_vertices()


        if tm_mesh.is_self_intersecting:
            self.get_logger().warning("Mesh is self-intersecting. Attempting to fix intersections with Trimesh.")
            # This can be computationally expensive and might alter the geometry significantly
            tm_mesh.process_intersections() # This attempts to resolve self-intersections

        # Re-check properties after trimesh repair
        self.get_logger().info("Checking mesh properties after Trimesh repair:")
        self.get_logger().info(f"Is watertight (trimesh): {tm_mesh.is_watertight}")
        self.get_logger().info(f"Is manifold (trimesh): {tm_mesh.is_manifold}")
        self.get_logger().info(f"Is self-intersecting (trimesh): {tm_mesh.is_self_intersecting}")

        # Convert back to Open3D mesh if needed, or work with trimesh directly for export
        # For exporting, trimesh can export directly to STL
        mesh_final = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(tm_mesh.vertices),
            o3d.utility.Vector3iVector(tm_mesh.faces)
        )
        mesh_final.compute_vertex_normals() # Recompute normals after any changes

        # Open3D's own checks (good for sanity check and logging)
        self.get_logger().info("Open3D mesh validation after repair:")
        self.get_logger().info(f"Open3D is_edge_manifold (allow_boundary_edges=True): {mesh_final.is_edge_manifold(allow_boundary_edges=True)}")
        self.get_logger().info(f"Open3D is_edge_manifold (allow_boundary_edges=False): {mesh_final.is_edge_manifold(allow_boundary_edges=False)}")
        self.get_logger().info(f"Open3D is_vertex_manifold: {mesh_final.is_vertex_manifold()}")
        self.get_logger().info(f"Open3D is_self_intersecting: {mesh_final.is_self_intersecting()}")
        self.get_logger().info(f"Open3D is_watertight: {mesh_final.is_watertight()}")
        self.get_logger().info(f"Open3D is_orientable: {mesh_final.is_orientable()}")


        self.get_logger().info("Exporting mesh")
        # Use trimesh's export to STL, as it's often more robust for repaired meshes
        # tm_mesh.export(self.directory + "-mesh-output.stl")
        # Or if you want to stick with Open3D for final export, use the converted mesh_final
        o3d.io.write_triangle_mesh(self.directory + "-mesh-output.stl", mesh_final)
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