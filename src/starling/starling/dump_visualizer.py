import os
import open3d as o3d

# Directory containing PCD files
pcd_directory = "trans_pointclouds"

# Load all PCD files from the directory
pcd_list = []
for filename in os.listdir(pcd_directory):
    if filename.endswith(".pcd"):
        pcd_path = os.path.join(pcd_directory, filename)
        pcd = o3d.io.read_point_cloud(pcd_path)
        pcd_list.append(pcd)

# Visualize the combined point clouds
o3d.visualization.draw_geometries(pcd_list)
