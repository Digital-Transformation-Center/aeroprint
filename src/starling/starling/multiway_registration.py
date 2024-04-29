import open3d as o3d
import numpy as np 

def load_point_clouds(voxel_size=0.0):
    # pcd_data = {
    #               0: "pointclouds/pointcloud33203106.pcd", 
    #               1: "pointclouds/pointcloud43272677.pcd", 
    #               2: "pointclouds/pointcloud53445894.pcd", 
    #               3: "pointclouds/pointcloud61447705.pcd", 
    #               4: "pointclouds/pointcloud138243516.pcd", 
    #               5: "pointclouds/pointcloud148293296.pcd", 
    #               6: "pointclouds/pointcloud158364951.pcd", 
    #               7: "pointclouds/pointcloud233202741.pcd", 
    #               8: "pointclouds/pointcloud243256948.pcd", 
    #               9: "pointclouds/pointcloud253355842.pcd", 
    #               10: "pointclouds/pointcloud261414424.pcd", 
    #               11: "pointclouds/pointcloud338248360.pcd", 
    #               12: "pointclouds/pointcloud348324129.pcd", 
    #               13: "pointclouds/pointcloud358366565.pcd", 
    #               14: "pointclouds/pointcloud433213001.pcd", 
    #               15: "pointclouds/pointcloud443294031.pcd"
    #             }
    pcd_data = {
        0: "trans_pointclouds/pointcloud3187374.pcd", 
        1: "trans_pointclouds/pointcloud12937919.pcd", 
        2: "trans_pointclouds/pointcloud212918939.pcd", 
        3: "trans_pointclouds/pointcloud403173066.pcd", 
        4: "trans_pointclouds/pointcloud612911614.pcd", 
        5: "trans_pointclouds/pointcloud811870852.pcd", 
        6: "trans_pointclouds/pointcloud1688670699.pcd", 
        7: "trans_pointclouds/pointcloud1888699312.pcd", 
        8: "trans_pointclouds/pointcloud2211018286.pcd"
    }
    pcds = []
    for i in range(8):
        pcd = o3d.io.read_point_cloud(pcd_data[i])
        # Remove points outside of a radius of 1.5m (mostly to get rid of walls and ceiling)
        points = np.asarray(pcd.points)
        center = np.array([0, 0, 0])
        radius = 15
        distances = np.linalg.norm(points - center, axis=1)
        pcd.points = o3d.utility.Vector3dVector(points[distances <= radius])
        # Filter out some noise with radius outlier technique (there are others that we can use.)
        pcd.remove_radius_outlier(nb_points=16, radius=0.05)
        # Downsample and estimate normals.
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        pcds.append(pcd_down)
    return pcds

def pairwise_registration(source, target):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id])
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph

voxel_size = 0.02
pcds_down = load_point_clouds(voxel_size)
# o3d.visualization.draw_geometries(pcds_down,
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])
print("Full registration ...")
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    pose_graph = full_registration(pcds_down,
                                   max_correspondence_distance_coarse,
                                   max_correspondence_distance_fine)

print("Transform points and display")
for point_id in range(len(pcds_down)):
    print(pose_graph.nodes[point_id].pose)
    pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
o3d.visualization.draw_geometries(pcds_down,
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])