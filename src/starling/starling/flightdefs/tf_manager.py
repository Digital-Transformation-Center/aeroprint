import rclpy
from rclpy.node import Node
import math
import numpy as np
from transforms3d.euler import quat2euler, euler2quat # For converting between quaternion and Euler angles
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_ros
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs # For transforming geometry messages

class TransformManager(Node):
    def __init__(self):
        super().__init__('transform_manager')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.parent_frame = 'world'
        self.child_frame = 'odom'

        self.home_transform_published = False
        self.home_position = None
        self.home_orientation = None

    def publish_home_transform(self):
        if not self.home_transform_published and self.home_position is not None and self.home_orientation is not None:
            self.get_logger().info('Attempting to publish static transform for starling_home...')
            if type(self.home_orientation) is float:
                orientation_quat_wxyz = euler2quat(0.0, 0.0, self.home_orientation)
            else:
                orientation_quat_wxyz = self.home_orientation

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.child_frame
            t.transform.translation.x = self.home_position[0]
            t.transform.translation.y = self.home_position[1]
            t.transform.translation.z = self.home_position[2]
            t.transform.rotation.x = orientation_quat_wxyz[1]
            t.transform.rotation.y = orientation_quat_wxyz[2]
            t.transform.rotation.z = orientation_quat_wxyz[3]
            t.transform.rotation.w = orientation_quat_wxyz[0]

            self.static_broadcaster.sendTransform(t)

    def wait_for_home_transform(self):
        """
        Wait for the static transform 'starling_home' to be available.
        This is useful if you need to ensure the transform is published before using it.
        """
        while not self.home_transform_published:
            # self.publish_home_transform()
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, rclpy.time.Time())
                self.home_transform_published = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Transform not found yet: {e}")
                continue
            self.get_logger().info('Home transform is now available.')
            return
        self.get_logger().info('Home transform was already published.')

    def set_home(self, position, orientation):
        self.home_position = [float(x) for x in position]
        self.home_orientation = float(orientation)
        self.home_transform_published = False
        self.publish_home_transform()
        self.get_logger().info(f'Home position set to: {position}, orientation set to: {orientation}')
    
    def clear(self):
        """
        Clear the home position and orientation.
        This will stop publishing the static transform.
        """
        self.home_position = None
        self.home_orientation = None
        self.home_transform_published = False
        self.get_logger().info('Home position and orientation cleared.')


    def get_vector_wrt_world(self, vector):
        """
        Get a vector in the world frame.
        Applies only rotation.
        :param vector: A 3D vector (e.g., [x, y, z]) in the 'starling_home' frame.
        :return: The vector in the world frame, or None if transform fails.
        """
        v = Vector3Stamped()
        v.header.stamp = self.get_clock().now().to_msg() # Use current time for lookup
        v.header.frame_id = self.child_frame
        v.vector.x = float(vector[0])
        v.vector.y = float(vector[1])
        v.vector.z = float(vector[2])
        try:
            transformed_vector = self.tf_buffer.transform(v, self.parent_frame, timeout=rclpy.duration.Duration(seconds=0.06))
            return [float(transformed_vector.vector.x), float(transformed_vector.vector.y), float(transformed_vector.vector.z)]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Error getting transform for vector: {e}")
            return None

    def get_position_wrt_world(self, position):
        """
        Get a position (point) in the world frame.
        Applies both rotation and translation.
        :param position: A 3D position (e.g., [x, y, z]) in the 'starling_home' frame.
        :return: The position in the world frame, or None if transform fails.
        """
        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self.child_frame
        p.point.x = position[0]
        p.point.y = position[1]
        p.point.z = position[2]
        try:
            transformed_position = self.tf_buffer.transform(p, 'world', timeout=rclpy.duration.Duration(seconds=0.06))
            return [float(transformed_position.point.x), float(transformed_position.point.y), float(transformed_position.point.z)]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Error getting transform for position: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    tm = TransformManager()
    # Set home to [0.0, 0.0, 0.0] in world, with 180 deg yaw rotation
    # This means starling_home's origin is at world(0,0,0) and its Y-axis points along world's -Y axis
    tm.set_home([0.0, 0.0, 0.0], math.pi / 4)

    try:
        rclpy.spin(tm)
    except KeyboardInterrupt:
        pass
    finally:
        tm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()