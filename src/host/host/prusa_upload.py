# Library import
import PrusaLinkPy

# Printer instantiation
prusaMK4 = PrusaLinkPy.PrusaLinkPy("192.168.8.181", "vWDzCjgQmUxfemt")

# Get bed temperature
getPrint = prusaMK4.get_printer()

# Display bed temperature
# print("Printer bed temperature: " + getPrint.json()["telemetry"]["temp-bed"])

# Transferring a file to the printer
if not prusaMK4.exists_gcode('/PrusaLink/miniJet.bgcode'):
    prusaMK4.put_gcode('/home/brian/aeroprint/src/host/host/miniJet.bgcode', '/PrusaLink/miniJet.bgcode', printAfterUpload=True, overwrite=False)
    print('file added')
else:
    print('file already exists')


# # List files on USB Drive
# prusaMK4.get_files().json()["children"]

# # Print this file 
try:
    prusaMK4.post_gcode('/PrusaLink/miniJet.bgcode')
    print("Heating up!")
except:
    print("Prusa MK4 failed to post gcode to print.")