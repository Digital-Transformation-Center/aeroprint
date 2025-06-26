# odometry_tf_publisher.py
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry # Make sure this message type is available in your ROS 2 workspace

class OdometryToTFPublisher(Node):
    """
    Subscribes to PX4 VehicleOdometry and publishes the dynamic transform
    from the 'odom' (world) frame to the drone's 'base_link' frame.
    """
    def __init__(self):
        super().__init__('odometry_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.initial_position = None  # Will be numpy array
        self.initial_orientation = None  # Will be numpy quaternion (w, x, y, z)
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.get_logger().info('Odometry to TF Publisher Node Started.')

    def odometry_callback(self, msg: VehicleOdometry):
        """
        Callback function for incoming VehicleOdometry messages.
        Publishes odom->base_link as the pose relative to the initial pose (odom).
        """
        import numpy as np
        from scipy.spatial.transform import Rotation as R

        # Get current pose
        pos = np.array([msg.position[0].item(), msg.position[1].item(), msg.position[2].item()])
        q = np.array([msg.q[0].item(), msg.q[1].item(), msg.q[2].item(), msg.q[3].item()])  # (w, x, y, z)

        if self.initial_position is None:
            self.initial_position = pos
            self.initial_orientation = q
            # Precompute inverse rotation for efficiency
            self.initial_rot_inv = R.from_quat([q[1], q[2], q[3], q[0]]).inv()  # scipy uses (x, y, z, w)
            self.get_logger().info('Saved initial odometry pose for odom frame.')

        # Compute relative translation (in initial frame)
        rel_pos = pos - self.initial_position
        rel_pos = self.initial_rot_inv.apply(rel_pos)

        # Compute relative rotation: q_rel = q_init_inv * q_curr
        curr_rot = R.from_quat([q[1], q[2], q[3], q[0]])
        rel_rot = self.initial_rot_inv * curr_rot
        rel_quat = rel_rot.as_quat()  # (x, y, z, w)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = rel_pos[0]
        t.transform.translation.y = rel_pos[1]
        t.transform.translation.z = rel_pos[2]
        t.transform.rotation.x = rel_quat[0]
        t.transform.rotation.y = rel_quat[1]
        t.transform.rotation.z = rel_quat[2]
        t.transform.rotation.w = rel_quat[3]

        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info(f'Published relative transform: {t.child_frame_id} from {t.header.frame_id}')

def main(args=None):
    rclpy.init(args=args)
    node = OdometryToTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
