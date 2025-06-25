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

        # Subscribe to the PX4 VehicleOdometry topic
        # Ensure the QoS profile matches the publisher's QoS if possible for reliability.
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            rclpy.qos.qos_profile_sensor_data # Use sensor data QoS for odometry
        )
        self.get_logger().info('Odometry to TF Publisher Node Started.')

    def odometry_callback(self, msg: VehicleOdometry):
        """
        Callback function for incoming VehicleOdometry messages.
        Extracts pose information and publishes it as a TF transform.
        """
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg() # Use current ROS time for TF message
        # PX4 Odometry data can be in a different timestamp domain (e.g., microseconds since boot).
        # For tf2, it's generally best to use the ROS time associated with when the message is processed.
        # If strict time synchronization with the odometry message's internal timestamp is needed,
        # you might convert msg.timestamp to rclpy.Time().
        # Example: t.header.stamp = rclpy.time.Time(nanoseconds=msg.timestamp * 1000).to_msg()
        
        t.header.frame_id = 'odom'       # Parent frame: typically the world or fixed origin for odometry
        t.child_frame_id = 'base_link'   # Child frame: the drone's body/center

        # Populate translation from the odometry message
        t.transform.translation.x = msg.position[0]
        t.transform.translation.y = msg.position[1]
        t.transform.translation.z = msg.position[2]

        # Populate rotation (quaternion) from the odometry message
        # PX4 uses (w, x, y, z) order for quaternions in VehicleOdometry
        t.transform.rotation.w = msg.q[0]
        t.transform.rotation.x = msg.q[1]
        t.transform.rotation.y = msg.q[2]
        t.transform.rotation.z = msg.q[3]

        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info(f'Published transform: {t.child_frame_id} from {t.header.frame_id}')

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
