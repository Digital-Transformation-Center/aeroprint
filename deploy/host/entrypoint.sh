source /opt/ros/humble/setup.bash
source /opt/px4_msgs/setup.bash
sudo colcon build --symlink-install
source install/setup.bash
# ros2 run host flask_server_node
ros2 launch host web_test.launch.py
# ros2 run host test_node