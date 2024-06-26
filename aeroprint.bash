#!/bin/bash

if [ "$HOSTNAME" = "m0054" ]; then
    # Your code here
    colcon build --packages-select starling
else
    colcon build --packages-select host printer
fi

# Relative path to the file from the script
relative_path="install/setup.bash"

# Directory of the current script
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Full path by combining script directory with the relative path
setup_path="${script_dir}/${relative_path}"

# Resolve the absolute path
# absolute_path=$("${full_path}")
source $setup_path

if [ "$HOSTNAME" = "m0054" ]; then
    ros2 launch starling starling_launch.py
else
    ros2 launch host host_launch.py
fi


