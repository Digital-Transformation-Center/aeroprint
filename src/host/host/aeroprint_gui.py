#!/usr/bin/env python3
"""
aeroprint_gui.py: PyQt GUI with ROS node for initiating scans.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.01.01"
__status__ = "Beta"


#!/usr/bin/env python3

import sys
import os
import rclpy
import threading
import math
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
    QSplashScreen,
    QGridLayout,
    QDoubleSpinBox,
    QCheckBox
)
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen, QColor
from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge  # if needed for ROS <-> OpenCV

ENABLE_SPLASH = True
SPLASH_TIME = 1000 # ms (1s)
DTC_BLUE = QColor(0, 189, 247)
DTC_RED = QColor(232, 37, 41)
BLACK = QColor(0, 0, 0)

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AEROPRINT")

        # Build App layout
        central_widget = QWidget()
        self.layout = QGridLayout(central_widget)
        self.qvio_widget = QVIOWidget()
        self.qvio_widget.setMinimumWidth(800)
        central_widget.setMinimumHeight(600)
        self.qvio_widget.resize(800, 800)

        self.g2r = GUItoROS(self.qvio_widget)  # The ROS node
        self.parameter_widget = ParameterWidget(self.g2r)
        self.parameter_widget.resize(800, 800)

        self.layout.addWidget(self.parameter_widget, 0, 1)
        self.layout.addWidget(self.qvio_widget, 0, 0)

        self.setCentralWidget(central_widget)
        self.setGeometry(100, 100, 1600, 900)

        self.flashSplash()
        self.start_ros_thread()

    def flashSplash(self):
        """
        If desired, show a splash screen image for a brief moment.
        """
        if ENABLE_SPLASH:
            # Example: if you have a splash.png file:
            splash_path = os.path.join(os.path.dirname(__file__), "splash.png")
            try:
                self.splash = QSplashScreen(QPixmap(splash_path))
                self.splash.show()
                QTimer.singleShot(SPLASH_TIME, self.splash.close)
            except:
                pass

    def start_ros_thread(self, args=None):
        # Spin the ROS node in a separate thread
        ros_thread = threading.Thread(target=self.update_from_ros)
        ros_thread.start()

    def update_from_ros(self):
        rclpy.spin(self.g2r)  # spin the node
        self.g2r.destroy_node()
        rclpy.shutdown()

    def show(self):
        if ENABLE_SPLASH:
            QTimer.singleShot(SPLASH_TIME, super().show)
        else:
            super().show()

class ParameterWidget(QWidget):
    """
    The main parameter panel. 
    Users can set radius, object height, start height, offset X/Y, then press 'Ready' or 'Land'.
    """
    def __init__(self, g2r):
        super().__init__()
        self.g2r = g2r

        # 1. UI controls
        self.scan_name_textbox = QLineEdit(self)
        self.scan_name_textbox.setMaxLength(25)

        self.ready_button = QPushButton('Ready', self)
        self.land_button = QPushButton('Land', self)

        self.radius_spin_box = QDoubleSpinBox(self)
        self.radius_spin_box.setDecimals(1)
        self.radius_spin_box.setRange(0.0, 10.0)  
        self.radius_spin_box.setValue(0.0)

        self.height_spin_box = QDoubleSpinBox(self)
        self.height_spin_box.setDecimals(2)
        self.height_spin_box.setRange(0.0, 5.0)

        self.start_height_spin_box = QDoubleSpinBox(self)
        self.start_height_spin_box.setDecimals(2)
        self.start_height_spin_box.setRange(0.0, 5.0)

        self.print_checkbox = QCheckBox("Send to printer when done", self)
        self.scan_title = ""

        # NEW: Offsets for circle center
        self.offset_x_spin_box = QDoubleSpinBox(self)
        self.offset_x_spin_box.setDecimals(2)
        self.offset_x_spin_box.setRange(-10.0, 10.0)
        self.offset_x_spin_box.setValue(0.0)

        self.offset_y_spin_box = QDoubleSpinBox(self)
        self.offset_y_spin_box.setDecimals(2)
        self.offset_y_spin_box.setRange(-10.0, 10.0)
        self.offset_y_spin_box.setValue(0.0)

        self.start_description = QLabel('')

        # 2. Connect signals
        self.ready_button.clicked.connect(self.ready)
        self.land_button.clicked.connect(self.land)
        self.radius_spin_box.valueChanged.connect(self.update_radius)
        self.height_spin_box.valueChanged.connect(self.update_height)
        self.start_height_spin_box.valueChanged.connect(self.update_start_height)
        self.scan_name_textbox.textChanged.connect(self.update_scan_title)
        self.print_checkbox.stateChanged.connect(self.print_check)
        self.offset_x_spin_box.valueChanged.connect(self.update_offset_x)
        self.offset_y_spin_box.valueChanged.connect(self.update_offset_y)

        # 3. Layout
        layout = QVBoxLayout()
        layout.addWidget(QLabel('Scan Title:'))
        layout.addWidget(self.scan_name_textbox)

        layout.addWidget(QLabel('Circle Radius (m):'))
        layout.addWidget(self.radius_spin_box)

        layout.addWidget(QLabel('Object Height (m):'))
        layout.addWidget(self.height_spin_box)

        layout.addWidget(QLabel('Object Starting Height (m):'))
        layout.addWidget(self.start_height_spin_box)

        layout.addWidget(QLabel('Circle Center Offset X (m):'))
        layout.addWidget(self.offset_x_spin_box)
        layout.addWidget(QLabel('Circle Center Offset Y (m):'))
        layout.addWidget(self.offset_y_spin_box)

        layout.addWidget(self.print_checkbox)
        layout.addWidget(self.start_description)
        layout.addWidget(self.ready_button)
        layout.addWidget(self.land_button)

        self.setLayout(layout)
        self.init_vals()
        self.update_start_description()

    def init_vals(self):
        """
        Initialize default publishers to ensure the drone node sees something.
        """
        self.g2r.publish_flight_radius(0.0)
        self.g2r.publish_object_height(0.0)
        self.g2r.publish_start_height(0.0)
        self.g2r.publish_offset_x(0.0)
        self.g2r.publish_offset_y(0.0)
        self.g2r.publish_ready(False)
        self.g2r.publish_scan_title("")
        self.g2r.publish_kill(False)
        self.g2r.publish_will_print(False)

    def update_radius(self, value):
        self.g2r.publish_flight_radius(value)
        self.update_start_description()

    def update_height(self, value):
        self.g2r.publish_object_height(value)
        self.update_start_description()

    def update_start_height(self, value):
        self.g2r.publish_start_height(value)
        self.update_start_description()

    def update_scan_title(self, value):
        self.scan_title = value
        self.g2r.publish_scan_title(value)
        self.update_start_description()

    def update_offset_x(self, value):
        self.g2r.publish_offset_x(value)

    def update_offset_y(self, value):
        self.g2r.publish_offset_y(value)

    def update_start_description(self):
        r = self.radius_spin_box.value()
        h = self.height_spin_box.value()
        s = self.start_height_spin_box.value()
        self.start_description.setText(
            f"Radius={r:.1f}m, Height={h:.2f}m, Start={s:.2f}m"
        )

    def print_check(self, val):
        self.g2r.publish_will_print(bool(val))

    def ready(self):
        # Publish final values
        r = self.radius_spin_box.value()
        h = self.height_spin_box.value()
        s = self.start_height_spin_box.value()
        self.g2r.publish_flight_radius(r)
        self.g2r.publish_object_height(h)
        self.g2r.publish_start_height(s)
        self.g2r.publish_scan_title(self.scan_title)
        # Then set ready True
        self.g2r.publish_ready(True)

    def land(self):
        self.g2r.publish_ready(False)

class QVIOWidget(QWidget):
    """
    Stub widget that can display an image from a /qvio_overlay ROS topic if desired.
    """
    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout()
        self.image_label = QLabel(self)
        self.image_label.resize(800, 600)
        self.setLayout(layout)

    def update_image_display(self, image_data):
        self.image_label.setPixmap(image_data)

class GUItoROS(rclpy.node.Node):
    """
    ROS node that publishes the user's GUI parameters to relevant topics.
    Also can subscribe to an image topic if needed.
    """
    def __init__(self, qvio_widget: QWidget) -> None:
        super().__init__("gui_node")
        self.qvio_widget = qvio_widget

        # Publishers
        self.flight_radius_pub = self.create_publisher(Float32, "/host/gui/out/radius", 10)
        self.object_height_pub = self.create_publisher(Float32, "/host/gui/out/object_height", 10)
        self.start_height_pub = self.create_publisher(Float32, "/host/gui/out/start_height", 10)
        self.scan_title_pub = self.create_publisher(String, "/host/gui/out/scan_title", 10)
        self.offset_x_pub = self.create_publisher(Float32, "/host/gui/out/offset_x", 10)
        self.offset_y_pub = self.create_publisher(Float32, "/host/gui/out/offset_y", 10)
        self.ready_pub = self.create_publisher(Bool, "/host/gui/out/ready", 10)
        self.kill_pub = self.create_publisher(Bool, "/host/gui/out/kill", 10)
        self.will_print_pub = self.create_publisher(Bool, "/host/gui/out/will_print", 10)

        # If you want to display a live image from a topic:
        # from rclpy.qos import qos_profile_sensor_data
        # self.qvio_sub = self.create_subscription(
        #     Image,
        #     "/qvio_overlay",
        #     self.qvio_callback,
        #     qos_profile_sensor_data
        # )

    def qvio_callback(self, image_msg: Image):
        """
        If you want to display an image in the GUI.
        Example conversion to QPixmap:
        """
        pass
        # try:
        #     # Convert image_msg to QImage -> QPixmap
        #     self.qvio_widget.update_image_display(self.pixmap)
        # except Exception as e:
        #     self.get_logger().info("Error parsing QVIO")

    # -------------------------------------------------
    # Publish convenience methods
    # -------------------------------------------------
    def publish_flight_radius(self, val):
        msg = Float32()
        msg.data = val
        self.flight_radius_pub.publish(msg)
        self.get_logger().info(f"Publishing radius: {val}")

    def publish_object_height(self, val):
        msg = Float32()
        msg.data = val
        self.object_height_pub.publish(msg)
        self.get_logger().info(f"Publishing object_height: {val}")

    def publish_start_height(self, val):
        msg = Float32()
        msg.data = val
        self.start_height_pub.publish(msg)
        self.get_logger().info(f"Publishing start_height: {val}")

    def publish_offset_x(self, val):
        msg = Float32()
        msg.data = val
        self.offset_x_pub.publish(msg)
        self.get_logger().info(f"Publishing offset_x: {val}")

    def publish_offset_y(self, val):
        msg = Float32()
        msg.data = val
        self.offset_y_pub.publish(msg)
        self.get_logger().info(f"Publishing offset_y: {val}")

    def publish_scan_title(self, val):
        msg = String()
        msg.data = val
        self.scan_title_pub.publish(msg)
        self.get_logger().info(f"Publishing scan_title: {val}")

    def publish_ready(self, val):
        msg = Bool()
        msg.data = val
        self.ready_pub.publish(msg)
        self.get_logger().info(f"Publishing ready: {val}")

    def publish_kill(self, val):
        msg = Bool()
        msg.data = val
        self.kill_pub.publish(msg)
        self.get_logger().info(f"Publishing kill: {val}")

    def publish_will_print(self, val):
        msg = Bool()
        msg.data = val
        self.will_print_pub.publish(msg)
        self.get_logger().info(f"Publishing will_print: {val}")

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()



