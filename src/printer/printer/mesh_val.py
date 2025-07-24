import open3d as o3d
import numpy as np
import trimesh as tm
import os
import argparse

def check_and_repair_mesh(input_stl_path: str, output_directory: str = None):
    """
    Loads an STL mesh, performs manifold, watertight, and self-intersection checks,
    attempts to repair the mesh using trimesh, and saves the repaired mesh.

    Args:
        input_stl_path (str): The file path to the input STL mesh.
        output_directory (str, optional): The directory to save the repaired STL.
                                          If None, it defaults to the directory of the input STL.
    """
    if not os.path.exists(input_stl_path):
        print(f"Error: Input STL file not found at '{input_stl_path}'")
        return

    print(f"\n--- Processing Mesh: {os.path.basename(input_stl_path)} ---")

    try:
        # Load the mesh using trimesh. Trimesh is often more robust for initial loading
        # of potentially problematic STLs.
        mesh = tm.load(input_stl_path)
        if not mesh:
            print(f"Error: Could not load mesh from {input_stl_path}")
            return
        if not mesh.is_empty:
            print(f"Loaded mesh with {len(mesh.vertices)} vertices and {len(mesh.faces)} faces.")
        else:
            print("Loaded an empty or invalid mesh.")
            return


    except Exception as e:
        print(f"Error loading mesh with trimesh: {e}")
        # Try loading with open3d as a fallback, though trimesh is generally preferred for this
        try:
            mesh_o3d = o3d.io.read_triangle_mesh(input_stl_path)
            if not mesh_o3d.has_triangles():
                print(f"Error: Open3D could not load a valid mesh from {input_stl_path}")
                return
            mesh = tm.Trimesh(vertices=np.asarray(mesh_o3d.vertices),
                              faces=np.asarray(mesh_o3d.triangles),
                              vertex_normals=np.asarray(mesh_o3d.vertex_normals))
            print("Successfully loaded with Open3D and converted to Trimesh.")
        except Exception as e_o3d:
            print(f"Error loading mesh with Open3D fallback: {e_o3d}")
            return


    print("\n--- Initial Mesh Properties (Trimesh) ---")
    print(f"Is watertight: {mesh.is_watertight}")
    print(f"Is manifold: {mesh.is_manifold}")
    print(f"Is self-intersecting: {mesh.is_self_intersecting}")
    print(f"Has faces: {mesh.has_faces}")
    print(f"Has connectivity: {mesh.is_connected}")
    # Additional common properties
    print(f"Euler number: {mesh.euler_number}") # For a manifold mesh without holes, this should be 2 for a sphere, 0 for a torus, etc.
    print(f"Vertex count: {len(mesh.vertices)}")
    print(f"Face count: {len(mesh.faces)}")


    print("\n--- Attempting Mesh Repair ---")

    # Attempt to fill holes
    if not mesh.is_watertight:
        print("Mesh is not watertight. Attempting to fill holes...")
        mesh.fill_holes()
        if mesh.is_watertight:
            print("Successfully filled holes and made mesh watertight.")
        else:
            print("Mesh still not watertight after fill_holes. Trying `repair.make_watertight` (more aggressive).")
            # This function tries to make the mesh watertight by attempting to fill holes and ensuring consistent orientation
            # Note: This can sometimes significantly alter the mesh's geometry.
            try:
                mesh = tm.repair.make_watertight(mesh)
                if mesh.is_watertight:
                    print("Successfully made mesh watertight with `repair.make_watertight`.")
                else:
                    print("Warning: Mesh remains not watertight after aggressive repair.")
            except Exception as repair_e:
                print(f"Error during `repair.make_watertight`: {repair_e}")


    # Remove non-manifold edges and vertices
    if not mesh.is_manifold:
        print("Mesh is not manifold. Attempting to remove non-manifold components...")
        initial_edges = len(mesh.edges_unique)
        initial_vertices = len(mesh.vertices)

        mesh.remove_nonmanifold_edges()
        mesh.remove_nonmanifold_vertices()

        if len(mesh.edges_unique) < initial_edges:
            print(f"Removed {initial_edges - len(mesh.edges_unique)} non-manifold edges.")
        if len(mesh.vertices) < initial_vertices:
            print(f"Removed {initial_vertices - len(mesh.vertices)} non-manifold vertices.")

        if mesh.is_manifold:
            print("Mesh is now manifold.")
        else:
            print("Warning: Mesh remains non-manifold after attempted repair of edges and vertices.")

    # Process self-intersections
    if mesh.is_self_intersecting:
        print("Mesh is self-intersecting. Attempting to fix intersections...")
        # `process_intersections` attempts to resolve self-intersections by splitting faces.
        # This can be computationally expensive and might modify the mesh significantly.
        try:
            mesh.process_intersections()
            if not mesh.is_self_intersecting:
                print("Successfully resolved self-intersections.")
            else:
                print("Warning: Mesh remains self-intersecting after `process_intersections`.")
        except Exception as intersect_e:
            print(f"Error during `process_intersections`: {intersect_e}")


    print("\n--- Final Mesh Properties (Trimesh) After Repair ---")
    print(f"Is watertight: {mesh.is_watertight}")
    print(f"Is manifold: {mesh.is_manifold}")
    print(f"Is self-intersecting: {mesh.is_self_intersecting}")
    print(f"Euler number: {mesh.euler_number}")
    print(f"Vertex count: {len(mesh.vertices)}")
    print(f"Face count: {len(mesh.faces)}")


    # Optionally, convert to Open3D mesh to perform Open3D's own checks
    print("\n--- Open3D Validation (After Trimesh Repair) ---")
    try:
        mesh_o3d_final = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(mesh.vertices),
            o3d.utility.Vector3iVector(mesh.faces)
        )
        # Ensure normals are computed for accurate checks by Open3D
        if not mesh_o3d_final.has_vertex_normals():
            mesh_o3d_final.compute_vertex_normals()

        print(f"Open3D is_edge_manifold (allow_boundary_edges=True): {mesh_o3d_final.is_edge_manifold(allow_boundary_edges=True)}")
        print(f"Open3D is_edge_manifold (allow_boundary_edges=False): {mesh_o3d_final.is_edge_manifold(allow_boundary_edges=False)}")
        print(f"Open3D is_vertex_manifold: {mesh_o3d_final.is_vertex_manifold()}")
        print(f"Open3D is_self_intersecting: {mesh_o3d_final.is_self_intersecting()}")
        print(f"Open3D is_watertight: {mesh_o3d_final.is_watertight()}")
        print(f"Open3D is_orientable: {mesh_o3d_final.is_orientable()}")
    except Exception as o3d_check_e:
        print(f"Could not perform Open3D validation: {o3d_check_e}")


    # Save the repaired mesh
    if output_directory is None:
        output_directory = os.path.dirname(input_stl_path)
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)

    base_name = os.path.splitext(os.path.basename(input_stl_path))[0]
    output_stl_path = os.path.join(output_directory, f"{base_name}_repaired.stl")

    try:
        mesh.export(output_stl_path)
        print(f"\nRepaired mesh saved to: {output_stl_path}")
    except Exception as e:
        print(f"Error saving repaired mesh: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Check and repair 3D meshes from an STL file."
    )
    parser.add_argument(
        "input_stl",
        type=str,
        help="Path to the input STL file."
    )
    parser.add_argument(
        "-o", "--output_dir",
        type=str,
        default=None,
        help="Optional: Directory to save the repaired STL. Defaults to input file's directory."
    )

    args = parser.parse_args()

    check_and_repair_mesh(args.input_stl, args.output_dir)

if __name__ == "__main__":
    main()
