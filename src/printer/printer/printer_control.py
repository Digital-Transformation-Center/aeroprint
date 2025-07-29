import subprocess
import PrusaLinkPy
import os
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rclpy.qos import qos_profile_system_default
import rclpy

class PrinterController(Node):
    def __init__(self) -> None:
        super().__init__("printer_control_node")

        self.file_directory_sub = self.create_subscription(
            String, 
            "/host/out/mesher/file_directory",
            self.file_directory_callback, 
            qos_profile_system_default
        )
        self.slicing_complete_pub = self.create_publisher(
            Bool, 
            "/host/out/slicer/slicing_complete",
            qos_profile_system_default
        )
        self.get_logger().info("Initializing Printer")
        self.prusaMK4 = PrusaLinkPy.PrusaLinkPy("192.168.8.181", "vWDzCjgQmUxfemt")
        self.printer = self.prusaMK4.get_printer()
        self.get_logger().info("Printer Initialized")
        self.gcode_path = ""
        self.filename = ""
        self.get_logger().info("Printer node alive!")

    def file_directory_callback(self, msg):
        self.get_logger().info("Recieved stl file path.")
        self.stl_file_path = msg.data
        self.get_logger().info("STL filer path: %s" % self.stl_file_path)
        output_directory = os.path.dirname(self.stl_file_path)
        base_filename = os.path.basename(self.stl_file_path)  # Extract the filename from the path
        filename_without_extension = os.path.splitext(base_filename)[0]  # Remove the extension
        if filename_without_extension.endswith("-output"):
            filename_without_extension = filename_without_extension[:-7]  # Remove the "-output" section
        self.filename = filename_without_extension
        self.gcode_path = os.path.join(output_directory, filename_without_extension) + ".gcode"
        self.get_logger().info("Gcode path: %s" % self.gcode_path)
        self.generate_gcode()
        self.print_gcode()

    def generate_gcode(self):
        self.get_logger().info("Beginning gcode generation.")
        path = os.path.dirname(os.path.abspath(__file__))  
        # Split the path into components
        path_parts = path.split(os.sep)

        # Find the index of 'aeroprint' in the path
        try:
            aeroprint_index = path_parts.index("aeroprint")
        except ValueError:
            print("Error: 'aeroprint' not found in the path")
        else:
            # Construct the truncated path
            truncated_path = os.sep.join(path_parts[:aeroprint_index + 1])
        max_size = 10.0
        config_dir = "src/printer/printer/printer_config_files/"
        config_path = os.path.join(truncated_path, "%sprusa_config.ini" % config_dir)
        start_path = os.path.join(truncated_path, "%sstart.gcode" % config_dir)
        end_path = os.path.join(truncated_path, "%send.gcode" % config_dir)
        berofe_path = os.path.join(truncated_path, "%sbefore_layer.gcode" % config_dir)
        
        command = "slic3r --load %s\
            --start-gcode %s\
            --end-gcode %s\
            --before-layer-gcode %s\
            --scale %.2f\
            --output %s %s " % (
                config_path, 
                start_path, 
                end_path, 
                berofe_path,
                max_size,
                self.gcode_path, 
                self.stl_file_path
            )
        subprocess.run(command, shell=True, executable="/bin/bash")
        self.slicing_complete_pub.publish(Bool(data=True))

    def print_gcode(self):
        printer_gcode_path = "/aeroprint/" + self.filename + ".gcode"
        self.get_logger().info("Initializing Printer")
        self.prusaMK4 = PrusaLinkPy.PrusaLinkPy("192.168.8.181", "vWDzCjgQmUxfemt")
        self.printer = self.prusaMK4.get_printer()
        self.get_logger().info("Printer Initialized")
        self.get_logger().info("Using file: " + self.gcode_path)
        self.get_logger().info("Attempting to place: " + printer_gcode_path)
        if self.prusaMK4.exists_gcode(printer_gcode_path):
            self.prusaMK4.delete(printer_gcode_path) 
        self.prusaMK4.put_gcode(self.gcode_path, printer_gcode_path)
        self.prusaMK4.post_print_gcode(printer_gcode_path)
        self.get_logger().info("GCode sent to printer.")

# path = os.path.dirname(os.path.abspath(__file__))  
# gcode_path = os.path.join(path, 'output.gcode')

def main(args = None) -> None:
    rclpy.init(args=args)
    printer_node = PrinterController()
    rclpy.spin(printer_node)
    printer_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


