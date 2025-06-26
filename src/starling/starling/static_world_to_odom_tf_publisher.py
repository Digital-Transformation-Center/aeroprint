# static_world_to_odom_tf_publisher.py
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry

class StaticWorldToOdomTFPublisher(Node):
    """
    Publishes a static transform from the 'world' frame to the 'odom' frame
    using the initial odometry data received from PX4 VehicleOdometry.
    """
    def __init__(self):
        super().__init__('static_world_to_odom_tf_publisher')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.odom_received = False
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.get_logger().info('Waiting for first odometry message to publish static world->odom transform...')

    def odometry_callback(self, msg: VehicleOdometry):
        if self.odom_received:
            return  # Only publish once
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        # Set translation to the initial odometry position
        t.transform.translation.x = msg.position[0].item()
        t.transform.translation.y = msg.position[1].item()
        t.transform.translation.z = msg.position[2].item()
        # Set rotation to the initial odometry orientation (PX4: q = [w, x, y, z])
        t.transform.rotation.w = msg.q[0].item()
        t.transform.rotation.x = msg.q[1].item()
        t.transform.rotation.y = msg.q[2].item()
        t.transform.rotation.z = msg.q[3].item()
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform from world to odom.')
        self.odom_received = True


def main(args=None):
    rclpy.init(args=args)
    node = StaticWorldToOdomTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
