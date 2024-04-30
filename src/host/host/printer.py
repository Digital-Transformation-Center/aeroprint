import subprocess
import PrusaLinkPy
import os

path = os.path.dirname(os.path.abspath(__file__))  
gcode_path = os.path.join(path, 'output.gcode')

prusaMK4 = PrusaLinkPy.PrusaLinkPy("192.168.8.181", "9BQHmQ5SRVc5RRE")

getPrint = prusaMK4.get_printer()
print(getPrint.json()["telemetry"])

if prusaMK4.exists_gcode("/usb/aeroprint/output.gcode"):
    prusaMK4.delete("/usb/aeroprint/output.gcode")
prusaMK4.put_gcode(gcode_path, '/usb/aeroprint/output.gcode')

prusaMK4.post_print_gcode('/usb/aeroprint/output.gcode')