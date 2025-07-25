from launch import LaunchDescription
from launch_ros.actions import Node
"""
The transform frames are:
- world: The global frame of reference, typically the ground or a fixed point.
- odom: The frame of reference relative to world with its zero point at the initial position of the drone.
- base_link: The frame of reference for the drone's body
- tof_sensor_frame: The frame of reference for the ToF sensor mounted on the drone.

world -> odom: Managed by tf_manager, built into the flight control node.
odom -> base_link: Managed by odometry-tf-publisher, which publishes the drone's position and orientation in the odom frame.
base_link -> tof_sensor_frame: Managed by static-tof-tf-publisher, which publishes the static transform between the base_link and tof_sensor_frame.
"""

def generate_launch_description():
    return LaunchDescription([
        # Web server node for Flask
        Node(package="host", executable="flask-server-node", name="web_server"), 
        # Flight control node
        Node(package="starling", executable="new-helix", name="helical_flight_node"),
        # Transform nodes for pointcloud data
        Node(package="starling", executable="odometry-tf-publisher", name="odometry_tf_pub"), 
        Node(package="starling", executable="point-cloud-transformer", name="point_cloud_transformer"),
        Node(package="starling", executable="static-tof-tf-publisher", name="static_tof_tf_publisher"),
        # tf_manager is now integrated into the flight node
        # --- Node(package="starling", executable="static-world-to-odom-tf-publisher", name="static_world_to_odom_tf_publisher"),

        Node(package="host", executable="pc-collection", name="point_cloud_collection_node"),
        Node(package="host", executable="pc-post-processor", name="pc_post_processor_node"),
        Node(package="host", executable="mesher", name="mesher_node"),

        # Node(package="printer", executable="printer-control", name="printer_node"),
        
    ])
