#!/usr/bin/env python3
"""
aeroprint_gui.py: PyQt GUI with ROS node for initiating scans.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.1.0"
__status__ = "Beta"

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget, QSplashScreen, QGridLayout, QDoubleSpinBox, QCheckBox
from PyQt5.QtGui import QPixmap, QPainter, QPen, QColor, QImage
from PyQt5.QtCore import QTimer, Qt
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import sys

ENABLE_SPLASH = True
SPLASH_TIME = 1000 # 3s splash screen
DTC_BLUE = QColor(0, 189, 247)
DTC_RED = QColor(232, 37, 41)
BLACK = QColor(0, 0, 0)

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AEROPRINT")

        # Build App
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

        print("Truncated path:", truncated_path)
        self.splash_path = os.path.join(truncated_path, 'src/host/images/splash.png')  
        print("PATH:")
        print(self.splash_path)
        self.flashSplash()
        central_widget = QWidget()
        self.layout = QGridLayout(central_widget)
        self.qvio_widget = QVIOWidget()
        self.qvio_widget.setMinimumWidth(800)
        central_widget.setMinimumHeight(600)
        self.qvio_widget.resize(800, 800)
        self.g2r = GUItoROS(self.qvio_widget)
        self.parameter_widget = ParameterWidget(self.g2r)
        self.parameter_widget.resize(800, 800)
        self.layout.addWidget(self.parameter_widget, 0, 1)
        self.layout.addWidget(self.qvio_widget, 0, 0)
        # central_widget.setLayout(self.layout)
        self.setCentralWidget(central_widget)
        self.start_ros_thread()

    def flashSplash(self):
        self.splash = QSplashScreen(QPixmap(self.splash_path))
        self.splash.show()
        QTimer.singleShot(SPLASH_TIME, self.splash.close)
    
    def start_ros_thread(self, args=None):
        # rclpy.init(args=args)
        ros_thread = threading.Thread(target=self.update_from_ros)
        ros_thread.start()

    def update_from_ros(self):
        rclpy.spin(self.g2r)
        self.g2r.destroy_node()
        rclpy.shutdown()

    def show(self):
        if ENABLE_SPLASH:
            self.flashSplash()
            QTimer.singleShot(SPLASH_TIME, super().show)
        else:
            super().show()
        
class ParameterWidget(QWidget):
    def __init__(self, g2r):
        super().__init__()
        self.g2r = g2r
        self.scan_name_textbox = QLineEdit(self)
        self.scan_name_textbox.setMaxLength(25)
        self.ready_button = QPushButton('Ready', self)
        self.land_button = QPushButton('Land', self)
        # Create a QDoubleSpinBox
        self.radius_spin_box = QDoubleSpinBox(self)
        self.radius_spin_box.setDecimals(1)  # Scan radius
        self.radius_spin_box.setRange(0.0, 3.0)  # Set the allowed value range        
        self.height_spin_box = QDoubleSpinBox(self)
        self.height_spin_box.setDecimals(2)  # Scan radius
        self.height_spin_box.setRange(0.0, 5.0)  # Set the allowed value range
        self.start_height_spin_box = QDoubleSpinBox(self)
        self.start_height_spin_box.setDecimals(2)  # Scan radius
        self.start_height_spin_box.setRange(0.0, 5.0)  # Set the allowed value range
        self.print_checkbox = QCheckBox("Send to printer when done", self)
        self.scan_title = ""
        self.radius = 0.0
        self.height = 0.0
        self.start_height = 0.0
        self.start_description = QLabel('')
        self.update_start_description()
        # Connect buttons to functions
        self.ready_button.clicked.connect(self.ready)
        self.land_button.clicked.connect(self.land)
        self.radius_spin_box.valueChanged.connect(self.update_radius)
        self.height_spin_box.valueChanged.connect(self.update_height)
        self.start_height_spin_box.valueChanged.connect(self.update_start_height)
        self.scan_name_textbox.textChanged.connect(self.update_scan_title)
        self.print_checkbox.stateChanged.connect(self.print_check)

        # Create layout
        layout = QVBoxLayout()
        layout.addWidget(QLabel('Scan Title:'))
        layout.addWidget(self.scan_name_textbox)
        layout.addWidget(QLabel('Scan Radius (From Center in Meters): '))
        layout.addWidget(self.radius_spin_box)
        # layout.addWidget(self.label)
        layout.addWidget(QLabel('Object Height (Approx. in Meters): '))
        layout.addWidget(self.height_spin_box)
        layout.addWidget(QLabel('Object Starting Height (From ground in Meters): '))
        layout.addWidget(self.start_height_spin_box)
        layout.addWidget(self.print_checkbox)
        layout.addWidget(self.start_description)
        layout.addWidget(self.ready_button)
        layout.addWidget(self.land_button)
        self.setLayout(layout)
        self.init_vals()

    def init_vals(self):
        self.g2r.publish_flight_radius(0.0)
        self.g2r.publish_kill(False)
        self.g2r.publish_object_height(0.0)
        self.g2r.publish_ready(False)
        self.g2r.publish_scan_title("")
        self.g2r.publish_start_height(0.0)
        self.g2r.publish_will_print(False)


    def print_check(self, val):
        self.g2r.publish_will_print(val)

    def update_start_description(self):
        self.start_description.setText("Start scan at {r:.1f}m for an object that is approximately {h:.2f}m tall, starting at {s:.2f}m above the ground.".format(r=self.radius, h=self.height, s=self.start_height))

    def update_radius(self, value):
        self.radius = value
        self.g2r.publish_flight_radius(value)
        self.update_start_description()

    def update_height(self, value):
        self.height = value
        self.g2r.publish_object_height(value)
        self.update_start_description()

    def update_start_height(self, value):
        self.start_height = value
        self.g2r.publish_start_height(value)
        self.update_start_description()

    def update_scan_title(self, value):
        self.scan_title = value
        self.g2r.publish_scan_title(value)
        self.update_start_description()

    def ready(self):
        self.g2r.publish_flight_radius(self.radius)
        self.g2r.publish_object_height(self.height)
        self.g2r.publish_start_height(self.start_height)
        self.g2r.publish_scan_title(self.scan_title)

        self.g2r.publish_ready(True)

    def land(self):
        self.g2r.publish_ready(False)

    def get_radius(self):
        return self.radius
    
    def get_height(self):
        return self.height

    def get_start_height(self):
        return self.start_height

    def get_scan_title(self):
        return self.scan_title
    
class QVIOWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()
        # self.setGeometry(100, 100, 800, 600)
        layout = QVBoxLayout()
        self.image_label = QLabel(self)
        self.image_label.resize(800, 600)
        # self.resize(800, 600)

        self.setLayout(layout)
        
    def update_image_display(self, image_data):
    # Update the QLabel with the received image data
    # You can process the image_data here (e.g., convert to QPixmap)
    # For simplicity, assume image_data is a QPixmap
        self.image_label.setPixmap(image_data)

    

class GUItoROS(Node):
    def __init__(self, qvio_widget:QWidget) -> None:
        super().__init__("gui_node")
        self.qvio_widget = qvio_widget
        # Define Publishers
        self.flight_radius_pub = self.create_publisher(
            Float32, 
            "/host/gui/out/radius",
            qos_profile_system_default
        )
        self.object_height_pub = self.create_publisher(
            Float32, 
            "/host/gui/out/object_height", 
            qos_profile_system_default
        )
        self.start_height_pub = self.create_publisher(
            Float32, 
            "/host/gui/out/start_height", 
            qos_profile_system_default
        )
        self.scan_title_pub = self.create_publisher(
            String, 
            "/host/gui/out/scan_title", 
            qos_profile_system_default
        )
        self.ready_pub = self.create_publisher(
            Bool, 
            "/host/gui/out/ready", 
            qos_profile_system_default
        )
        self.kill_pub = self.create_publisher(
            Bool, 
            "/host/gui/out/kill", 
            qos_profile_system_default
        )
        self.will_print_pub = self.create_publisher(
            Bool, 
            "/host/gui/out/will_print", 
            qos_profile_system_default
        )
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.qvio_sub = self.create_subscription(
            Image, 
            "/qvio_overlay", 
            self.qvio_callback, 
            qos_profile=qos_profile_sensor_data
        )

    def qvio_callback(self, image_msg: Image):
        img_format_table = {'rgb8': QImage.Format_RGB888, 'mono8': QImage.Format_Grayscale8, 'yuv422': QImage.Format_BGR30}
        try:
            self.format = img_format_table[image_msg.encoding]
            self.qimage = QImage(image_msg.data, image_msg.width, image_msg.height, self.format)
            self.pixmap = QPixmap.fromImage(self.qimage)
            self.qvio_widget.update_image_display(self.pixmap)

        except Exception as e:
            self.get_logger().info("Error parsing QVIO")

    def publish_will_print(self, val):
        self.will_print_pub.publish(self.create_bool(bool(val)))
        self.info("Publishing will print: " + str(bool(val)))
    
    def publish_object_height(self, height):
        self.object_height_pub.publish(self.create_float32(height))
        self.info("Publishing object height: " + str(height))

    def publish_flight_radius(self, rad):
        self.flight_radius_pub.publish(self.create_float32(rad))
        self.info("Publishing flight radius: " + str(rad))

    def publish_start_height(self, height):
        self.start_height_pub.publish(self.create_float32(height))
        self.info("Publishing start height: " + str(height))

    def publish_scan_title(self, title):
        self.scan_title_pub.publish(self.create_string(title))
        self.info("Publishing scan title: " + str(title))
    
    def publish_ready(self, ready):
        self.ready_pub.publish(self.create_bool(ready))
        self.info("Publishing ready: " + str(ready))
    
    def publish_kill(self, kill):
        self.kill_pub.publish(self.create_bool(kill))
        self.info("Publishing kill: " + str(kill))

    def create_float32(self, val):
        res = Float32()
        res.data = val
        return res
    
    def create_bool(self, val):
        res = Bool()
        res.data = val
        return res

    def create_string(self, val):
        res = String()
        res.data = val
        return res
    
    def info(self, text):
        self.get_logger().info(text)

class HeightGraphic(QWidget):
    def __init__(self):
        self.height = 300
        self.width = 400
        
        super(HeightGraphic, self).__init__()
        self.setFixedSize(self.height, self.width)  # Set canvas size
        self.start_point = (100, 150)  # Initial start point
        self.end_point = (300, 150)  # Initial end point

    def paintEvent(self, e):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw a line using the start and end points
        painter.setPen(QPen(DTC_BLUE, 2, Qt.SolidLine))
        painter.drawLine(*self.start_point, *self.end_point)

    def set_line_endpoints(self, start_point, end_point):
        self.start_point = start_point
        self.end_point = end_point
        self.update()  # Update the canvas


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = MyWindow()

    # Show the main window
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()