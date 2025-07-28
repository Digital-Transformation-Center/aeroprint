from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Web server node for Flask
        Node(package="host", executable="flask-server-node", name="web_server"), 
        
    ])
