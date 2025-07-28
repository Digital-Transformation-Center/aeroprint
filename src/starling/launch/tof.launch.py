from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="starling", executable="odometry-tf-publisher", name="odometry_tf_pub"), 
            Node(package="starling", executable="point-cloud-transformer", name="point_cloud_transformer"),
            Node(package="starling", executable="static-tof-tf-publisher", name="static_tof_tf_publisher"),
            Node(package="starling", executable="static-world-to-odom-tf-publisher", name="static_world_to_odom_tf_publisher"),
            Node(package="starling", executable="static-cam-tf-publisher", name="static_cam_tf_publisher"),
        ]
    )
