from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="voxl_mpa_to_ros2",
                  executable="voxl_mpa_to_ros2_node",
                    name="voxl_mpa"),
            Node(package="starling", executable="odometry-tf-publisher", name="odometry_tf_pub"), 
            Node(package="starling", executable="point-cloud-transformer", name="point_cloud_transformer"),
            Node(package="starling", executable="static_tof_tf_publisher", name="static_tof_tf_publisher"),
        ]
    )
