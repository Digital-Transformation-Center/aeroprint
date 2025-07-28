import open3d as o3d
import numpy as np
import trimesh as tm
import os
import argparse

def voxel_repair(mesh, pitch=None):
    """
    Reconstructs a watertight mesh by voxelizing the input mesh and
    extracting the surface with marching cubes.
    """
    try:
        print("\nAttempting voxel-based repair...")

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

        print("Voxel repair succeeded.")
        return watertight_mesh

    except Exception as e:
        print(f"Voxel repair failed: {e}")
        return None


def check_and_fix_watertightness(input_stl_path: str, output_directory: str = None):
    """
    Loads an STL mesh, checks and attempts to fix watertightness,
    and saves the repaired mesh if possible.
    """
    if not os.path.exists(input_stl_path):
        print(f"Error: Input STL file not found at '{input_stl_path}'")
        return

    print(f"\n--- Processing Mesh for Watertightness: {os.path.basename(input_stl_path)} ---")

    try:
        mesh = tm.load(input_stl_path, force='mesh')
        if not isinstance(mesh, tm.Trimesh):
            raise ValueError("Loaded object is not a Trimesh mesh.")
        print(f"Loaded mesh with {len(mesh.vertices)} vertices and {len(mesh.faces)} faces.")
    except Exception as e:
        print(f"Error loading mesh: {e}")
        return

    print(f"Initial watertight: {mesh.is_watertight}")

    # Initial cleaning
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices()
    tm.repair.fill_holes(mesh)

    print(f"Watertight after basic cleanup: {mesh.is_watertight}")

    # Try voxel repair if still not watertight
    if not mesh.is_watertight:
        voxel_mesh = voxel_repair(mesh)
        if voxel_mesh and voxel_mesh.is_watertight:
            mesh = voxel_mesh
        else:
            print("Mesh remains non-watertight after all attempts.")
            return

    # Final cleanup
    mesh.remove_degenerate_faces()
    mesh.fix_normals()
    mesh.remove_infinite_values()

    # Output path setup
    if output_directory is None:
        output_directory = os.path.dirname(input_stl_path)
    os.makedirs(output_directory, exist_ok=True)

    output_path = os.path.join(
        output_directory,
        os.path.splitext(os.path.basename(input_stl_path))[0] + "_watertight_fixed.stl"
    )

    try:
        mesh.export(output_path)
        print(f"Final mesh is watertight: {mesh.is_watertight}")
        print(f"Saved repaired mesh to: {output_path}")
    except Exception as e:
        print(f"Error saving repaired mesh: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Check and fix watertightness of a 3D mesh from an STL file."
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
    check_and_fix_watertightness(args.input_stl, args.output_dir)


if __name__ == "__main__":
    main()
