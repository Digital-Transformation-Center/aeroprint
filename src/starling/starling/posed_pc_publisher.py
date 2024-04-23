import rclpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default, QoSProfile, HistoryPolicy

from rclpy.node import Node
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
    self.publisher = self.create_publisher(
        PointCloud2, "/starling/out/posed_pc", qos_profile_sensor_data
    )
    self.pose_node = pose_node

    

  def callback(self, data):
    # Get latest pose data
    rclpy.spin_once(self.pose_node)
    pose = self.pose_node.get_pose()
    pose_time = pose.header.stamp.nanosec
    pc_time = data.header.stamp.nanosec
    time_dif = (pose_time - pc_time) / 1000000000.0
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
    self.publisher.publish(data)

class PoseNode(Node):
  def __init__(self) -> None:
    super().__init__("pose_handler_node")
    topic = "/vvhub_body_wrt_fixed"
    qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)
    self.sub = self.create_subscription(
      PoseStamped, 
      topic, 
      self.callback, 
      qos_profile
    )
    self.pose = None
  def callback(self, msg):
    self.pose = msg
  def get_pose(self):
    return self.pose

def main(args=None) -> None:
    rclpy.init(args=args)
    pose_node = PoseNode()
    pc_node = PCNode(pose_node)
    rclpy.spin(pc_node)
    pc_node.destroy_node()
    pose_node.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
    
