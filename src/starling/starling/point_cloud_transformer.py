"""Transform incoming ToF PointCloud2 into target frame with TF wait and fallback."""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import tf2_ros
from tf2_sensor_msgs import do_transform_cloud


class PointCloudTransformer(Node):
    """Point cloud transformer with TF wait and latest fallback."""

    def __init__(self) -> None:
        super().__init__('point_cloud_transformer')

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Parameters
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('wait_timeout_sec', 0.25)   # seconds to wait for exact-time TF
        self.declare_parameter('max_wait_tries', 2)        # attempts for exact-time TF
        self.declare_parameter('fallback_to_latest', True) # if exact-time fails, use latest

        # Subscriptions
        self.subscription = self.create_subscription(
            PointCloud2,
            '/tof_pc',
            self.point_cloud_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.get_logger().info('Subscribing to /tof_pc')

        # Publications
        self.publisher = self.create_publisher(
            PointCloud2,
            '/starling/out/relative_posed_pc',
            rclpy.qos.qos_profile_sensor_data
        )
        self.get_logger().info('Publishing to /starling/out/relative_posed_pc')

        # Flight start hook (optional refresh)
        self.start_flight_subscriber = self.create_subscription(
            Bool,
            '/start_flight',
            self.start_flight,
            10,
        )

        self.get_logger().info('Point Cloud Transformer Node Started.')

    def start_flight(self, msg: Bool) -> None:
        # Refresh TF buffer and listener when flight starts (optional)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        # Frames
        source_frame = msg.header.frame_id or 'tof_sensor_frame'
        target_frame = self.get_parameter('target_frame').value

        # Timing
        stamp = Time.from_msg(msg.header.stamp)
        wait_timeout = Duration(seconds=float(self.get_parameter('wait_timeout_sec').value))
        max_tries = int(self.get_parameter('max_wait_tries').value)
        use_latest_fallback = bool(self.get_parameter('fallback_to_latest').value)

        try:
            # Try exact-time TF
            transform = None
            for attempt in range(max_tries):
                if self.tf_buffer.can_transform(target_frame, source_frame, stamp, wait_timeout):
                    transform = self.tf_buffer.lookup_transform(target_frame, source_frame, stamp)
                    break
                self.get_logger().warn(
                    f"TF not available yet ({attempt + 1}/{max_tries}) for {source_frame}->{target_frame} at {stamp.nanoseconds}")

            # Fallback to latest TF
            if transform is None and use_latest_fallback:
                latest = Time()  # time 0 => latest available
                if self.tf_buffer.can_transform(target_frame, source_frame, latest, wait_timeout):
                    transform = self.tf_buffer.lookup_transform(target_frame, source_frame, latest)
                    ts = transform.header.stamp
                    self.get_logger().warn(
                        f"Using latest TF for {source_frame}->{target_frame} (stamp: {ts.sec}.{ts.nanosec:09d})")

            if transform is None:
                raise tf2_ros.ExtrapolationException(
                    f"No TF for {source_frame}->{target_frame} at {stamp.nanoseconds} and no latest fallback.")

            # Transform and publish
            transformed_cloud = do_transform_cloud(msg, transform)
            transformed_cloud.header.frame_id = target_frame
            transformed_cloud.header.stamp = msg.header.stamp
            self.publisher.publish(transformed_cloud)

        except tf2_ros.LookupException as ex:
            self.get_logger().warn(f'Transform lookup exception: {ex}')
        except tf2_ros.ConnectivityException as ex:
            self.get_logger().warn(f'Transform connectivity exception: {ex}')
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().warn(f'Transform extrapolation exception (data too old/new): {ex}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error transforming point cloud: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
