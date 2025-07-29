source /opt/ros/humble/setup.bash
source /home/aeroprint_user/external_packages/install/setup.bash

colcon build
# sudo colcon build --symlink-install --packages-select host starling
source install/setup.bash
# ros2 run host flask_server_node
# ros2 launch starling aeroprint.launch.py
ros2 launch starling aeroprint.launch.py

echo "Starting the host entrypoint script..."
# ros2 run host test_node