source /opt/ros/humble/setup.bash
source /home/aeroprint_user/external_packages/install/setup.bash

colcon build
source install/setup.bash

ros2 launch starling aeroprint.launch.py

echo "Starting the host entrypoint script..."
