from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="host", executable="flask-server-node", name="web_server"), 
        Node(package="host", executable="test-node", name="util_node"),
        Node(package="starling", executable="helical-flight", name="helical_flight_node"),
    ])
