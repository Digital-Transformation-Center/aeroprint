import numpy as np
import open3d as o3d
import copy
print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("pointclouds/pointcloud33203106.pcd")
points = np.asarray(pcd.points)
# Choose a reference point (e.g., origin)
reference_point = np.array([0.0, 0.0, 0.0])

# Calculate distances to the reference point
distances = np.linalg.norm(points - reference_point, axis=1)

# Filter points within 2 meters
filtered_points = points[distances <= 2.3]

# Update the point cloud
pcd.points = o3d.utility.Vector3dVector(filtered_points)
print(pcd)
print(np.asarray(pcd.points))

o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
# source = o3d.io.read_point_cloud("pointclouds/pointcloud33203106.pcd")
# target = o3d.io.read_point_cloud("pointclouds/pointcloud43272677.pcd")
# source.paint_uniform_color([1, 0.706, 0])
# target.paint_uniform_color([0, 0.651, 0.929])
# threshold = 0.02
# # trans_init = np.asarray([[1.0, 1.0, 1.0, 1.0], [1.0, 0.0, 0.0, 0.0],
# #                              [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
# trans_init = np.eye(4)
# trans = copy.deepcopy(target)
# trans.paint_uniform_color([0.651, 0, 0.929])

# for i in range(5):
#   reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init, o3d.pipelines.registration.TransformationEstimationPointToPoint())
#   print(reg_p2p)
#   print("Transformation is:")
#   print(reg_p2p.transformation) 
#   trans.transform(reg_p2p.transformation)
#   trans_init = reg_p2p.transformation
# o3d.visualization.draw_geometries([source, target, trans],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])