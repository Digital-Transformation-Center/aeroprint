import PrusaLinkPy
prusaMK4 = PrusaLinkPy.PrusaLinkPy("192.168.8.181", "9BQHmQ5SRVc5RRE")
printer = prusaMK4.get_printer()
gcode_path = "/home/ryan/aeroprint/scans/office-chair-2.gcode"
printer_gcode_path = "/aeroprint/office-chair-2.gcode"
print(printer.json()["telemetry"]["temp-bed"])
print("sending")
if prusaMK4.exists_gcode(printer_gcode_path):
    prusaMK4.delete(printer_gcode_path) 
prusaMK4.put_gcode(gcode_path, printer_gcode_path)
prusaMK4.post_print_gcode(printer_gcode_path)
print("done")