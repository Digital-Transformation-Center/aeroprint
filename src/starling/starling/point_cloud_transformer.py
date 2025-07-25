# point_cloud_transformer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import tf2_ros
from tf2_sensor_msgs import do_transform_cloud
# import tf2_sensor_msgs  # This module provides do_transform_cloud
from tf2_geometry_msgs import do_transform_pose_stamped # Not directly used for PC2, but useful to know

class PointCloudTransformer(Node):
    """
    Subscribes to raw ToF PointCloud2 data, looks up the transform from
    the ToF sensor frame to the world (odom) frame using tf2, transforms
    the point cloud, and publishes the result.
    """
    def __init__(self):
        super().__init__('point_cloud_transformer')

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber for the raw ToF point cloud data
        self.subscription = self.create_subscription(
            PointCloud2,
            '/tof_pc', # Original ToF point cloud topic
            self.point_cloud_callback,
            rclpy.qos.qos_profile_sensor_data # Use sensor data QoS
        )
        self.get_logger().info('Subscribing to /tof_pc')

        # Publisher for the transformed point cloud data
        self.publisher = self.create_publisher(
            PointCloud2,
            '/starling/out/relative_posed_pc', # New topic for transformed point cloud
            rclpy.qos.qos_profile_sensor_data
        )

        self.start_flight_subscriber = self.create_subscription(
            Bool, 
            "/start_flight",
            self.start_flight,
            10
        )
        self.get_logger().info('Publishing to /starling/out/relative_posed_pc')

        self.get_logger().info('Point Cloud Transformer Node Started.')

    def start_flight(self, msg: Bool):
        # Refresh the TF buffer and listener when flight starts
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def point_cloud_callback(self, msg: PointCloud2):
        """
        Callback function to process incoming point cloud data, transform it
        using tf2, and publish the result.
        """
        # Define the source and target frames for the transformation
        source_frame = msg.header.frame_id # The frame in which the incoming point cloud is defined (e.g., 'tof_sensor_frame')
        target_frame = 'odom'             # The desired output frame (the fixed world frame)

        # If the incoming message doesn't have a frame_id set, assume 'tof_sensor_frame'
        # if not source_frame:
        source_frame = 'tof_sensor_frame'
            # self.get_logger().warn(f"Incoming PointCloud2 message has no frame_id. Assuming '{source_frame}'.")

        try:
            # Look up the transform from the source frame to the target frame
            # at the timestamp of the incoming point cloud message.
            # Using msg.header.stamp for time travel to get the transform that was valid
            # when the point cloud data was captured.
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1) # Max wait time for transform
            )

            # Transform the point cloud
            transformed_cloud = do_transform_cloud(msg, transform)

            # Update the header of the transformed cloud
            transformed_cloud.header.frame_id = target_frame
            transformed_cloud.header.stamp = msg.header.stamp # Keep original timestamp if desired, or use current ROS time

            # Publish the transformed point cloud
            self.publisher.publish(transformed_cloud)
            # self.get_logger().info(f'Transformed and published point cloud from {source_frame} to {target_frame}.')

        except tf2_ros.LookupException as ex:
            self.get_logger().warn(f'Transform lookup exception: {ex}')
        except tf2_ros.ConnectivityException as ex:
            self.get_logger().warn(f'Transform connectivity exception: {ex}')
        except tf2_ros.ExtrapolationException as ex:
            # This can happen if the transform buffer doesn't have data for the requested timestamp
            # or if the data is too old/too new.
            self.get_logger().warn(f'Transform extrapolation exception (data too old/new): {ex}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')

def main(args=None):
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
