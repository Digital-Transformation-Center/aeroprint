#!/bin/bash

# parent_path=$(cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P)
# cd "$parent_path"
# while getopts ":mesh:" opt; do
#     case "$opt" in
#         mesh) MESH_PATH="$OPTARG";
#            ;;
#         \?) echo "Invalid option: -$OPTARG" >&2; exit 1; # Handle invalid options
#             ;;
#     esac
# done
# shift $((OPTIND-1))

slic3r --load printer_config_files/prusa_config.ini\
 --start-gcode printer_config_files/start.gcode\
  --end-gcode printer_config_files/end.gcode\
  --before-layer-gcode printer_config_files/before_layer.gcode\
   --output output.gcode ~/Downloads/pcd-output.stl 


