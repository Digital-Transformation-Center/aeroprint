# Library import
import PrusaLinkPy

# Printer instantiation
prusaMK4 = PrusaLinkPy.PrusaLinkPy('192.168.8.181', 'vWDzCjgQmUxfemt')
file = 'miniJet_54m.bgcode'
prusa_path = '/PrusaLink/' + file
host_path = '/home/brian/aeroprint/src/host/host/' + file

# Get bed temperature
getPrint = prusaMK4.get_printer()

# Display bed temperature
# print("Printer bed temperature: " + getPrint.json()["telemetry"]["temp-bed"])

# Transferring a file to the printer
if not prusaMK4.exists_gcode(prusa_path):
    prusaMK4.put_gcode(host_path, prusa_path, printAfterUpload=True, overwrite=False)
    print('file added')
else:
    print('file already exists')


# # List files on USB Drive
# prusaMK4.get_files().json()["children"]

# # Print this file 
try:
    prusaMK4.post_gcode(prusa_path)
    print("Heating up!")
except:
    print("Prusa MK4 failed to post gcode to print.")