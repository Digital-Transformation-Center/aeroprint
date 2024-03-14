from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='starling',
            executable='test_pub',
            name='drone'
        ),
        Node(
            package='host', 
            executable='test_sub', 
            name='host'
        )
    ])