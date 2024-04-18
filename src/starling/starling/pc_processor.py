import rclpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
from rclpy.qos import qos_profile_sensor_data

import time

class PCConverter():
  def __init__(self, args=None):
      # self.vis = o3d.visualization.Visualizer()
      # self.vis.create_window()
      rclpy.init(args=args)
      node = rclpy.create_node('pointcloud_converter')
      subscription = node.create_subscription(
          PointCloud2,
          '/tof_pc',
          self.callback,
          qos_profile_sensor_data
      )
      
      self.o3dpc = o3d.geometry.PointCloud()
      try: 
        rclpy.spin(node)
      except KeyboardInterrupt:
         o3d.io.write_point_cloud("pointclouds.pcd", self.o3dpc)
         print("done")


  def callback(self, data):
      # Extract point cloud data from ROS message
      try: 
        print("data @" + str(data.header.stamp.nanosec))
        points = np.frombuffer(data.data, dtype=np.float32).reshape(-1, 3)# [:, :3]
        o3dpc = o3d.geometry.PointCloud()
        o3dpc.points = o3d.utility.Vector3dVector(points)
        self.o3dpc += o3dpc
        # Now you have the Open3D point cloud 'o3dpc'
        o3d.io.write_point_cloud("pointclouds/pointcloud" + str(data.header.stamp.nanosec) + ".pcd", o3dpc)
        # o3d.io.write_point_cloud("pointclouds.pcd", o3dpc)
        # self.vis.add_geometry(o3dpc)
        # self.vis.poll_events()
        # self.vis.update_renderer()
      except KeyboardInterrupt:
        o3d.io.write_point_cloud("pointclouds.pcd", self.o3dpc)
        print("done")

def main():
    pcc = PCConverter()

    

if __name__ == '__main__':
    main()
    
