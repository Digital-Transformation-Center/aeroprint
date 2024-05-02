"""
custom_pc_publisher.py: ROS node for publishing PointCloud2, transformed for drone pose.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.1.0"
__status__ = "Beta"

import rclpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSProfile
from rclpy.node import Node
import numpy as np
import open3d as o3d
import threading


class PCNode(Node):
  """Node for publishing transformed point cloud data."""
  def __init__(self, pose_node) -> None:
    print("new PC Node.")
    super().__init__("point_cloud_handler_node")
    topic = "/tof_pc"
    qos_profile = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,  # Set queue size to 1
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE
    )
    self.sub = self.create_subscription(
      PointCloud2, 
      topic, 
      self.callback, 
      qos_profile_sensor_data
    )
    self.publisher = self.create_publisher(
        PointCloud2, "/starling/out/posed_pc", qos_profile_sensor_data
    )
    self.pose_node = pose_node
    self.pub_rate = 4.0 #2 hz
    self.max_time_dif = 0.02
    self.last_pub_time = self.get_clock().now().nanoseconds



  def callback(self, data):
    """Transform and publish recieved PointCloud2 message."""
    try:
      pc_time = data.header.stamp.nanosec
      pose = self.pose_node.find(data.header.stamp.nanosec)
      pose_time = pose.header.stamp.nanosec
      time_dif = abs(pose_time - pc_time) / (1.0 * 1e9)
      foo = ""
      if (pose_time - pc_time > 0):
          foo = "new"
      else:
        foo = "old"
      print(str(time_dif) + ", " + foo + " pose")

      current_time = self.get_clock().now().nanoseconds

      if (current_time - self.last_pub_time) >= (1 / self.pub_rate) * 1e9 and time_dif <= self.max_time_dif:
        # Get latest pose data
        print(time_dif)
        # Get position and orientation from pose
        position = pose.pose.position
        orientation = pose.pose.orientation
        # Convert PointCloud2 to numpy and o3d
        points = np.frombuffer(data.data, dtype=np.float32).reshape(-1, 3)# [:, :3]
        for i in range(len(points)):
            points[i] = [points[i][2], -points[i][1], points[i][0]]
        o3dpc = o3d.geometry.PointCloud()
        o3dpc.points = o3d.utility.Vector3dVector(points)

        # Rotate and translate for position
        position = np.array([position.x, position.y, position.z], dtype=np.float64)
        orientation = np.array([orientation.w, orientation.x, orientation.y, orientation.z], dtype=np.float64)
        R = o3d.geometry.get_rotation_matrix_from_quaternion(orientation)
        o3dpc.rotate(R, center=(0, 0, 0))
        o3dpc.translate(position)

        # Convert back to PointCloud2
        points = np.asarray(o3dpc.points, dtype=np.float32)
        data.data = points.tobytes()
        data.header.stamp.nanosec = int(time_dif * 1e9)
        self.publisher.publish(data)
        self.last_pub_time = current_time
      else:
        print("BAD SCAN: " + str(time_dif))
    except Exception as e:
       self.get_logger().info(str(e))

class PoseNode(Node):
  """Node for acquiring pose data from VOXL MPA."""
  def __init__(self) -> None:
    super().__init__("pose_handler_node")
    topic = "/vvhub_body_wrt_fixed"
    self.sub = self.create_subscription(
      PoseStamped, 
      topic, 
      self.callback, 
      qos_profile_system_default
    )
    self.lookup = {}
    self.pose = None
  def callback(self, msg):
    """Add pose to dictionary."""
    self.get_logger().info("Recieved pose data.")
    self.pose = msg
    self.lookup[msg.header.stamp.nanosec] = msg
  def get_pose(self):
    """Get a pose"""
    return self.pose
  def find(self, target):
    """Find a pose with a key closest to a target"""
    absolute_differences = np.abs(np.array(list(self.lookup.keys())) - target)
    # Find the index of the minimum absolute difference
    closest_index = np.argmin(absolute_differences)
    closest_key = list(self.lookup.keys())[closest_index]
    closest_value = self.lookup[closest_key]
    keys_to_remove = list(self.lookup.keys())[:-10]
    for key in keys_to_remove:
        self.lookup.pop(key, None)
    return closest_value

def main(args=None) -> None:
    """Start both nodes in separate threads"""
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    pose_node = PoseNode()
    pc_node = PCNode(pose_node)
    executor.add_node(pose_node)
    executor.add_node(pc_node)
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = pose_node.create_rate(2)
    try:
        while rclpy.ok():
            print('Help me body, you are my only hope')
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()

    

if __name__ == '__main__':
    main()