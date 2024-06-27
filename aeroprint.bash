#!/bin/bash


# Relative path to the file from the script
relative_path="src/host/host/launch_gui.py"

# Directory of the current script
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Full path by combining script directory with the relative path
setup_path="${script_dir}/${relative_path}"

python3 $setup_path


