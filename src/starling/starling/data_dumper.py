import rclpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node
import time
import numpy as np
import open3d as o3d

class DataDumperNode(Node):
  def __init__(self) -> None:
    super().__init__("data_dumper_node")
    self.tof_pc_sub = self.create_subscription(
       PointCloud2, 
       "/tof_pc", 
       self.pc_callback,
       qos_profile_sensor_data
    )

    self.pose_sub = self.create_subscription(
      PoseStamped, 
      "/vvhub_body_wrt_fixed", 
      self.pose_callback, 
      qos_profile_system_default
    )

    self.last_pose = None
    self.pcd_list = []

  def pc_callback(self, data):
      points = np.frombuffer(data.data, dtype=np.float32).reshape(-1, 3)# [:, :3]
      for i in range(len(points)):
         points[i] = [points[i][2], -points[i][1], points[i][0]]
      o3dpc = o3d.geometry.PointCloud()
      o3dpc.points = o3d.utility.Vector3dVector(points)
      last_point = self.last_pose.pose.position
      last_orientation = self.last_pose.pose.orientation
      position = np.array([last_point.x, last_point.y, last_point.z], dtype=np.float64)
      orientation = np.array([last_orientation.w, last_orientation.x, last_orientation.y, last_orientation.z], dtype=np.float64)
      R = o3d.geometry.get_rotation_matrix_from_quaternion(orientation)
      # T = np.eye(4)
      # T[:3, :3] = R
      # T[:3, 3] = position
      # o3dpc.transform(T)
      o3dpc.rotate(R, center=(0, 0, 0))
      o3dpc.translate(position)
      # self.pcd_list.append(o3dpc)
      o3d.io.write_point_cloud("trans_pointclouds/pointcloud" + str(data.header.stamp.nanosec) + ".pcd", o3dpc)
      print("PointCloud2: " + str(data.header.stamp.nanosec) + ", Last Pose: " + str(self.last_pose.header.stamp.nanosec))


  def pose_callback(self, data):
     self.last_pose = data
     print("UPDATING POSITION DATA")
    #  print("Pose: " + str(data.header.stamp.nanosec))hho

def main(args=None) -> None:
    rclpy.init(args=args)
    data_dumper_node = DataDumperNode()
    rclpy.spin(data_dumper_node)
    data_dumper_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
    
