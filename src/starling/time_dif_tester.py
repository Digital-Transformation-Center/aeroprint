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

class PCNode(Node):
  def __init__(self, pose_node) -> None:
    print("new PC Node.")
    super().__init__("point_cloud_handler_node")
    topic = "/tof_pc"
    self.sub = self.create_subscription(
      PointCloud2, 
      topic, 
      self.callback, 
      qos_profile_sensor_data
    )
    self.pose_node = pose_node
    self.pub_rate = 2.0 #2 hz
    self.max_time_dif = 5000000
    self.last_pub_time = self.get_clock().now().nanoseconds

  def callback(self, data):
    current_time = self.get_clock().now().nanoseconds
    rclpy.spin_once(self.pose_node)
    pose = self.pose_node.get_pose()
    pose_time = pose.header.stamp.nanosec
    pc_time = data.header.stamp.nanosec
    time_dif = abs(pose_time - pc_time)
    if (current_time - self.last_pub_time) >= (1 / self.pub_rate) * 1e9 and time_dif <= self.max_time_dif:        
        print(time_dif)
        self.last_pub_time = current_time
    else:
        print("conditions not met... time difference: " + str(time_dif))

class PoseNode(Node):
  def __init__(self) -> None:
    super().__init__("pose_handler_node")
    topic = "/vvhub_body_wrt_fixed"
    self.sub = self.create_subscription(
      PoseStamped, 
      topic, 
      self.callback, 
      qos_profile_system_default
    )
    self.pose = None
  def callback(self, msg):
    self.pose = msg
  def get_pose(self):
    return self.pose
  

def main(args=None) -> None:
    rclpy.init(args=args)
    # data_dumper_node = DataDumperNode()
    # rclpy.spin(data_dumper_node)
    # data_dumper_node.destroy_node()
    pose_node = PoseNode()
    pc_node = PCNode(pose_node)
    rclpy.spin(pc_node)
    pc_node.destroy_node()
    pose_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
    
