from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="host", executable="gui_test", name="host"), 
        Node(package="host", executable="pc-collection", name="host"), 
        Node(package="host", executable="pc-post-processor", name="host"), 
        Node(package="host", executable="mesher", name="host"), 
        Node(package="printer", executable="printer-control", name="printer")
    ])
