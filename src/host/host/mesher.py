import open3d as o3d
import numpy as np
import trimesh as tm

pcd_path = "first_test_points/test6/combined_filtered.pcd"
pcd = o3d.io.read_point_cloud(pcd_path)
pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
radii = [0.005, 0.01, 0.02, 0.04]
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
    pcd, 0.02)
# Test comment
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#     pcd, o3d.utility.DoubleVector(radii))
# mesh.compute_vertex_normals()
# # mesh = mesh.filter_smooth_simple(number_of_iterations=5)
mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)
# mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh).fill_holes().to_legacy()

trimesh_mesh = tm.Trimesh(vertices=mesh.vertices, faces=mesh.triangles)
trimesh_mesh.fill_holes()
trimesh_mesh.export("trimesh_mesh.stl")

# print('run Poisson surface reconstruction')
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         pcd, depth=20)
print(mesh)

# o3d.visualization.draw_geometries([pcd, mesh])
