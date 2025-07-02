# static_tof_tf_publisher.py
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

class StaticToFTransformPublisher(Node):
    """
    Publishes a static transform between the drone's base_link and the ToF sensor frame.
    
    This transform represents the fixed physical offset and orientation of the
    ToF sensor relative to the drone's central reference point (base_link).
    
    YOU MUST CUSTOMIZE THE TRANSLATION AND ROTATION VALUES BELOW BASED ON YOUR
    MODALAI STARLING'S EXTRINSICS.CONF FILE FOR THE TOF SENSOR.
    """
    def __init__(self):
        super().__init__('static_tof_tf_publisher')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.publish_static_transform)
        self.get_logger().info('Static ToF Transform Publisher Node Started.')

    def publish_static_transform(self):
        """
        Constructs and publishes the static transform.
        """
        t = TransformStamped()

        # Set the header. This transform is static, so the timestamp can be 0,
        # or it can be the current time if you want it to appear in tf trees
        # immediately upon startup. For static transforms, rclpy.time.Time() is fine.
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Parent frame: drone's center
        t.child_frame_id = 'camera_frame' # Child frame: camera

        # Translation from base_link to camera_frame (in meters)
        # Assuming sensor is mounted on the front, facing forward, slightly below center.
        translation = [-0.06, 0, 0.01]
        rpy_degrees = [0, 0, -90]
        rotation_scipy = R.from_euler('xyz', rpy_degrees, degrees=True)  # Convert roll, pitch, yaw to rotation matrix

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        r_quat = rotation_scipy.as_quat()  # Convert rotation matrix to quaternion (x, y, z, w)
        t.transform.rotation.x = r_quat[0]
        t.transform.rotation.y = r_quat[1]
        t.transform.rotation.z = r_quat[2]
        t.transform.rotation.w = r_quat[3]


        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info(f'Published static transform: {t.child_frame_id} from {t.header.frame_id}')

def main(args=None):
    rclpy.init(args=args)
    node = StaticToFTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
