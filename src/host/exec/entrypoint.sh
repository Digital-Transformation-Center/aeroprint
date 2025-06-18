source /opt/ros/humble/setup.bash
# source /home/aeroprint_user/external_packages/install/setup.bash

colcon build
# sudo colcon build --symlink-install --packages-select host starling
source install/setup.bash
# export QT_QPA_PLATFORM=offscreen
# ros2 run host flask_server_node
ros2 launch host host_launch.py
echo "Starting the host entrypoint script..."
# ros2 run host test_node