import rclpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node
import numpy as np
import open3d as o3d

class PCNode(Node):
  def __init__(self) -> None:
    print("new PC Node.")
    # Create the node and subscriber
    super().__init__("point_cloud_handler_node")
    topic = "/starling/out/posed_pc"
    self.sub = self.create_subscription(
      PointCloud2, 
      topic, 
      self.callback, 
      qos_profile_sensor_data
    )

    self.dump_dir = "first_test_points/test6" # Dump directory
    self.pc_interval = 1.0 # Point cloud interval in seconds
    # Keep a timestamp to limit scan frequency
    self.last_pc = self.get_clock().now().nanoseconds

  def callback(self, data):
    now = self.get_clock().now().nanoseconds
    if now - self.last_pc >= self.pc_interval * 1e9:
        print(data.header.stamp.nanosec) # For debugging
        # Convert ROS PointCloud2 to Open 3D point cloud
        points = np.frombuffer(data.data, dtype=np.float32).reshape(-1, 3)# [:, :3]
        o3dpc = o3d.geometry.PointCloud()
        o3dpc.points = o3d.utility.Vector3dVector(points)

        # Write point clouds to files
        o3d.io.write_point_cloud(self.dump_dir + "/pointcloud" + str(data.header.stamp.nanosec) + ".pcd", o3dpc)
        self.last_pc = now # Update time

def main(args=None) -> None:
    rclpy.init(args=args)
    pc_node = PCNode()
    rclpy.spin(pc_node)
    pc_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
    
