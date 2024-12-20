#!/usr/bin/env python3
"""
aeroprint_gui.py: PyQt GUI with ROS node for initiating scans.
UDRI DTC AEROPRINT
"""
__author__ = "Ryan Kuederle"
__email__ = "ryan.kuederle@udri.udayton.edu"
__version__ = "0.01.01"
__status__ = "Beta"

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget, QSplashScreen, QGridLayout, QDoubleSpinBox, QCheckBox, QComboBox
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
# from cv_bridge import CvBridge
import threading
import sys
import time
from host.gui.gui_pages.tensor_flow.ml_ui import MLUI

ENABLE_SPLASH = True
SPLASH_TIME = 1000 # 3s splash screen
DTC_BLUE = QColor(0, 189, 247)
DTC_RED = QColor(232, 37, 41)
BLACK = QColor(0, 0, 0)
VIDEO_URL = "http://192.168.8.191/video_raw/hires_small_color"
recording_circle = None

class FlightWidget(QWidget):
    """
    A widget for the AEROPRINT application that integrates various components 
    such as image exporting, parameter configuration, and ROS communication.
    
    Attributes:

        resources_path (str): Path to the resources directory.
        scroll_container (QWidget): The scroll container widget.
        mlui (object): An instance of the MLUI class.
        splash_path (str): Path to the splash image.
        image_exporter (ImageExporter): An instance of the ImageExporter class.
        layout (QGridLayout): The layout manager for the widget.
        qvio_widget (QVIOWidget): The QVIO widget for visual odometry.
        g2r (GUItoROS): An instance of the GUItoROS class for ROS communication.
        parameter_widget (ParameterWidget): The widget for parameter configuration.
        splash (QSplashScreen): The splash screen widget.
    """

    def __init__(self, resources_path, scroll_container, mlui) -> None:
        super().__init__()
        self.scroll_container = scroll_container
        self.resources_path = resources_path
        self.mlui = mlui
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
        self.image_exporter = ImageExporter(self.resources_path)
        self.layout = QGridLayout(self)
        self.qvio_widget = QVIOWidget(self.image_exporter)
        self.qvio_widget.setMinimumWidth(800)
        self.setMinimumHeight(600)
        self.qvio_widget.resize(800, 800)
        self.g2r = GUItoROS(self.qvio_widget, self.image_exporter, self.scroll_container, self.mlui)
        self.parameter_widget = ParameterWidget(self.g2r, self.image_exporter, self.resources_path)
        self.parameter_widget.resize(800, 800)
        self.layout.addWidget(self.parameter_widget, 0, 1)
        self.layout.addWidget(self.qvio_widget, 0, 0)
        # central_widget.setLayout(self.layout)
        # self.setCentralWidget(central_widget)
        self.start_ros_thread()

    def flashSplash(self):
        """
        Displays a splash screen for a specified duration.
        This method creates a QSplashScreen using the image specified by 
        `self.splash_path`, shows it, and then closes it after a predefined 
        amount of time (`SPLASH_TIME`).

        Attributes:

            self.splash_path (str): Path to the splash screen image.
            SPLASH_TIME (int): Duration in milliseconds for which the splash 
                       screen is displayed.
        """

        self.splash = QSplashScreen(QPixmap(self.splash_path))
        self.splash.show()
        QTimer.singleShot(SPLASH_TIME, self.splash.close)
    
    def start_ros_thread(self, args=None):
        """
        Starts a new thread to update from ROS (Robot Operating System).
        This method initializes a new thread that runs the `update_from_ros` method.
        Optionally, it can take arguments to initialize the ROS client library.

        Args:

            args (list, optional): A list of command-line arguments to initialize the ROS client library. Defaults to None.
        """

        # rclpy.init(args=args)
        ros_thread = threading.Thread(target=self.update_from_ros)
        ros_thread.start()

    def update_from_ros(self):
        """
        Spins the ROS node to process incoming messages, then destroys the node and shuts down the ROS client library.
        
        This method performs the following steps:

        1. Spins the ROS node to process incoming messages.
        2. Destroys the ROS node.
        3. Shuts down the ROS client library.

        Note:

            This method blocks until the node is destroyed.
        """

        rclpy.spin(self.g2r)
        self.g2r.destroy_node()
        rclpy.shutdown()

    def show(self):
        """
        Displays the widget. If the splash screen is enabled, it shows the splash screen
        for a specified duration before displaying the main widget.
        If the splash screen is not enabled, it directly displays the main widget.
        """

        if ENABLE_SPLASH:
            self.flashSplash()
            QTimer.singleShot(SPLASH_TIME, super().show)
        else:
            super().show()
        
class ParameterWidget(QWidget):
    """
    A widget for configuring and managing flight parameters for a scanning operation.
    This widget allows the user to select a dataset, input scan details such as title, radius, height, and starting height,
    and control the readiness and landing state of the operation. It also provides visual feedback and updates to the user
    through various UI elements.
    
    Attributes:

        g2r: An object responsible for communication with the flight controller.
        image_exporter: An object responsible for exporting images.
        resources_path: The path to the resources directory.
        dataset_dropdown: A dropdown menu for selecting datasets.
        dataset_name_textbox: A textbox for entering a custom dataset name.
        scan_name_textbox: A textbox for entering the scan title.
        ready_button: A button to indicate readiness for the scan.
        land_button: A button to land the flight.
        radius_spin_box: A spin box for setting the scan radius.
        height_spin_box: A spin box for setting the object height.
        start_height_spin_box: A spin box for setting the starting height of the object.
        start_description: A label displaying the start description of the scan.
        scan_title: The title of the scan.
        radius: The radius of the scan.
        height: The height of the object being scanned.
        start_height: The starting height of the object above the ground.
    """

    def __init__(self, g2r, image_exporter, resources_path):
        super().__init__()
        # This is for testing
        self.image_exporter = image_exporter
        self.resources_path = resources_path

        self.g2r = g2r
        ### Dataset Selection
        self.dataset_dropdown = QComboBox(self)
        self.dataset_dropdown.addItem("Select Dataset")
        # Get the list of folders in the dataset class
        dataset_folders = os.listdir(os.path.join(self.resources_path, 'datasets'))
        # Add each folder to the dropdown
        for folder in dataset_folders:
            self.dataset_dropdown.addItem(folder)
        # Add a textbox as the last item in the dropdown
        self.dataset_dropdown.addItem("Other")
        self.dataset_dropdown.currentIndexChanged.connect(self.dataset_selection_changed)
        self.dataset_name_textbox = QLineEdit(self)
        self.dataset_name_textbox.setMaxLength(25)
        self.dataset_name_textbox.textChanged.connect(self.update_dataset_name)
        self.dataset_name_textbox.hide()
        ### --------------------------------
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
        # self.print_checkbox = QCheckBox("Send to printer when done", self)
        global recording_circle
        recording_circle = CircleWidget()
        recording_circle.set_size(50, 50)
        recording_circle.set_color(DTC_BLUE)
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
        # self.print_checkbox.stateChanged.connect(self.print_check)

        # Create layout
        layout = QVBoxLayout()
        # layout.addWidget(QLabel('Select Dataset:'))
        layout.addWidget(self.dataset_dropdown)
        layout.addWidget(self.dataset_name_textbox)
        layout.addWidget(QLabel('Scan Title:'))
        layout.addWidget(self.scan_name_textbox)
        layout.addWidget(QLabel('Scan Radius (From Center in Meters): '))
        layout.addWidget(self.radius_spin_box)
        # layout.addWidget(self.label)
        layout.addWidget(QLabel('Object Height (Approx. in Meters): '))
        layout.addWidget(self.height_spin_box)
        layout.addWidget(QLabel('Object Starting Height (From ground in Meters): '))
        layout.addWidget(self.start_height_spin_box)
        # layout.addWidget(self.print_checkbox)
        layout.addWidget(recording_circle)
        layout.addWidget(self.start_description)
        layout.addWidget(self.ready_button)
        layout.addWidget(self.land_button)
        self.setLayout(layout)
        self.init_vals()

    def dataset_selection_changed(self, index):
        """
        Handles the event when the dataset selection is changed in the dropdown.
        
        Args:

            index (int): The index of the selected item in the dataset dropdown.
        
        Behavior:

            - If the selected index is the last item in the dropdown, the dropdown is disabled,
              the current index is set to 0, and the dataset name textbox is shown.
            - Otherwise, updates the dataset name based on the currently selected text in the dropdown.
        """

        if index == self.dataset_dropdown.count() - 1:
            self.dataset_dropdown.setDisabled(True)
            self.dataset_dropdown.setCurrentIndex(0)
            
            self.dataset_name_textbox.show()
        else:
            self.update_dataset_name(self.dataset_dropdown.currentText())

    def update_dataset_name(self, name):
        """
        Updates the dataset name in the GUI and notifies relevant components.
        
        Args:

            name (str): The new name of the dataset.
        
        This method updates the dataset name in the image exporter and publishes
        the new dataset name to the scan dataset publisher.
        """

        # self.dataset_dropdown.addItem(name)
        # self.dataset_dropdown.setCurrentIndex(self.dataset_dropdown.count() - 2)
        # self.dataset_name_textbox.hide()
        # self.dataset_dropdown.setDisabled(False)
        self.image_exporter.update_dataset(name)
        self.g2r.publish_scan_ds(name)

    def init_vals(self):
        """
        Initialize the default values for the flight widget.
        This method sets the initial state of various parameters by publishing
        default values to the corresponding topics.

        - Flight radius is set to 0.0.
        - Kill switch is set to False.
        - Object height is set to 0.0.
        - Ready state is set to False.
        - Scan title is set to an empty string.
        - Start height is set to 0.0.
        - Will print flag is set to False.
        """

        self.g2r.publish_flight_radius(0.0)
        self.g2r.publish_kill(False)
        self.g2r.publish_object_height(0.0)
        self.g2r.publish_ready(False)
        self.g2r.publish_scan_title("")
        self.g2r.publish_start_height(0.0)
        self.g2r.publish_will_print(False)


    def print_check(self, val):
        """
        Publishes a message indicating whether printing will occur.
        
        Args:

            val (bool): A boolean value indicating whether printing will occur.
        """

        self.g2r.publish_will_print(val)

    def update_start_description(self):
        """
        Updates the start description text with formatted values for radius, height, and start height.
        
        The description text is set to:
        
        "Start scan at {r:.1f}m for an object that is approximately {h:.2f}m tall, starting at {s:.2f}m above the ground."
        
        The values are formatted as follows:
        - r: radius (formatted to 1 decimal place)
        - h: height (formatted to 2 decimal places)
        - s: start height (formatted to 2 decimal places)
        """

        self.start_description.setText("Start scan at {r:.1f}m for an object that is approximately {h:.2f}m tall, starting at {s:.2f}m above the ground.".format(r=self.radius, h=self.height, s=self.start_height))

    def update_radius(self, value):
        """
        Update the flight radius and publish the new value.
        This method updates the radius attribute with the given value,
        publishes the new flight radius to the g2r object, and updates
        the start description accordingly.
        
        Args:

            value (float): The new radius value to be set.
        """

        self.radius = value
        self.g2r.publish_flight_radius(value)
        self.update_start_description()

    def update_height(self, value):
        """
        Updates the height attribute and publishes the new height value.
        
        Args:

            value (float): The new height value to be set.
        """

        self.height = value
        self.g2r.publish_object_height(value)
        self.update_start_description()

    def update_start_height(self, value):
        """
        Updates the start height value and publishes it.
        
        Args:

            value (float): The new start height value.
        
        This method sets the start height to the given value, publishes the 
        new start height using the g2r object, and updates the start 
        description accordingly.
        """

        self.start_height = value
        self.g2r.publish_start_height(value)
        self.update_start_description()

    def update_scan_title(self, value):
        """
        Updates the scan title with the given value and performs related updates.
        
        Args:

            value (str): The new scan title to be set.
        
        This method updates the scan title, publishes the new title, updates the start description,
        and updates the class of the image exporter with the new title.
        """

        self.scan_title = value
        self.g2r.publish_scan_title(value)
        self.update_start_description()
        self.image_exporter.update_class(value)        

    def ready(self):
        """
        Publishes various flight parameters and sets the system to ready state.
        
        This method publishes the following parameters to the g2r (ground to rover) communication system:
        
        - Flight radius
        - Object height
        - Start height
        - Scan title
        
        After publishing these parameters, it sets the system to a ready state by publishing a ready signal.
        """


        self.g2r.publish_flight_radius(self.radius)
        self.g2r.publish_object_height(self.height)
        self.g2r.publish_start_height(self.start_height)
        self.g2r.publish_scan_title(self.scan_title)

        self.g2r.publish_ready(True)
        # self.image_exporter.set_dump_status(True, 'dataset0', 'class0')

    def land(self):
        """
        Prepares the system for landing by publishing a 'not ready' status and 
        disabling the image exporter's dump status.
        
        This method performs the following actions:
        
        1. Publishes a 'not ready' status to the g2r topic.
        2. Sets the image exporter's dump status to False.
        """

        self.g2r.publish_ready(False)
        self.image_exporter.set_dump_status(False)

    def get_radius(self):
        """
        Returns the radius value.
        
        Returns:
            
            float: The radius value.
        """
        
        return self.radius
    
    def get_height(self):
        """
        Returns the height of the flight widget.
        
        Returns:
            
            int: The height of the flight widget.
        """

        return self.height

    def get_start_height(self):
        """
        Retrieve the starting height.
        
        Returns:

            float: The starting height.
        """

        return self.start_height

    def get_scan_title(self):
        """
        Retrieve the title of the scan.
        
        Returns:

            str: The title of the scan.
        """

        return self.scan_title
    
class QVIOWidget(QWidget):
    """
    QVIOWidget is a custom QWidget that displays video frames from a video source.
    
    Attributes:

        image_exporter: An object responsible for exporting images.
        image_label: A QLabel widget to display the video frames.
        left: The left position of the widget.
        top: The top position of the widget.
        width: The width of the widget.
        height: The height of the widget.
        cap: A cv2.VideoCapture object to capture video frames.
        dump_images: A boolean indicating whether to dump images.
        img_num: An integer to keep track of the number of images.
    """

    def __init__(self, image_exporter) -> None:
        super().__init__()
        self.image_exporter = image_exporter
        # self.setGeometry(100, 100, 800, 600)
        layout = QVBoxLayout()
        self.image_label = QLabel(self)
        self.left, self.top, self.width, self.height = 100, 100, 800, 600
        self.image_label.resize(self.width, self.height)
        self.cap = cv2.VideoCapture(VIDEO_URL)
        self.dump_images = False
        # self.resize(800, 600)
        self.img_num = 0
        self.setLayout(layout)

    def update_frame(self):
        """
        Capture a frame from the video stream, process it, and update the GUI with the new frame.
        
        This method performs the following steps:
        
        1. Captures a frame from the video stream.
        2. If image export is enabled, exports the captured frame.
        3. Converts the frame from BGR to RGB format.
        4. Converts the RGB image to a QImage.
        5. Converts the QImage to a QPixmap.
        6. Scales the QPixmap to fit the widget's dimensions while maintaining the aspect ratio.
        7. Updates the image label with the new QPixmap.
        
        Returns:
            
            None
        """

        ret, frame = self.cap.read()
        if self.image_exporter.get_dump_status():
            self.image_exporter.export_image(frame)
        if ret:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            convert_to_qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(convert_to_qt_format)
            pixmap = pixmap.scaled(self.width, self.height, Qt.KeepAspectRatio)
            self.image_label.setPixmap(pixmap)

    def start_image_dump(self):
        """
        Enables the dumping of images by setting the dump_images attribute to True.
        This method is used to start the process of dumping images, which can be 
        useful for debugging or logging purposes.
        """

        self.dump_images = True

    def stop_image_dump(self):
        """
        Stops the image dumping process by setting the dump_images flag to False.
        """

        self.dump_images = False


        
    def update_image_display(self, image_data):
        """
        Updates the image display with the provided image data.
        
        Args:
            
            image_data (QPixmap): The image data to be displayed on the QLabel.
        """

    # Update the QLabel with the received image data
    # You can process the image_data here (e.g., convert to QPixmap)
    # For simplicity, assume image_data is a QPixmap
        self.image_label.setPixmap(image_data)

class ImageExporter():
    """
    A class to handle exporting images to a specified directory structure.
    Attributes:
        resources_path (str): The base path where resources are stored.
        dataset (str): The name of the dataset to which images belong.
        class_name (str): The name of the class within the dataset.
        img_num (int): The current image number to be exported.
        gap (int): The gap between image exports.
        ready_to_dump (bool): A flag indicating whether the exporter is ready to dump images.
        image_dump_path (str): The path where images will be dumped.
    """

    def __init__(self, resources_path):
        self.resources_path = resources_path
        self.dataset = 'dataset0'
        self.class_name = 'class0'
        self.update_export_dir()
        self.img_num = 0
        self.gap = 9
        self.ready_to_dump = False

    def export_image(self, image):
        if self.img_num % self.gap == 0:
            if not os.path.exists(self.image_dump_path):
                os.makedirs(self.image_dump_path)
            path = os.path.join(self.image_dump_path, 'img' + str(self.img_num) + '.jpg')
            print("Exporting image to: " + path)
            cv2.imwrite(path, image)

        self.img_num += 1

    def update_export_dir(self):
        self.image_dump_path = os.path.join(self.resources_path, 'datasets', self.dataset, self.class_name, 'images')

    def update_class(self, name):
        self.class_name = name
        self.update_export_dir()

    def update_dataset(self, name):
        self.dataset = name
        self.update_export_dir()

    def get_dataset(self):
        return self.dataset
    
    def get_class(self):
        return self.class_name

    def set_dump_status(self, status, dataset = None, class_name = None):
        if status:
            self.img_num = 0
            # self.update_dataset(dataset)
            # self.update_class(class_name)
        self.ready_to_dump = status
    
    def get_dump_status(self):
        return self.ready_to_dump

class GUItoROS(Node):
    """
    A ROS2 node that interfaces with a GUI to publish and subscribe to various topics.
    Attributes:
        qvio_widget (QWidget): The widget for visual odometry.
        image_exporter: The object responsible for exporting images.
        scroll_container: The container for scrolling through UI elements.
        mlui: The machine learning UI component.
        flight_radius_pub (Publisher): Publisher for flight radius.
        object_height_pub (Publisher): Publisher for object height.
        start_height_pub (Publisher): Publisher for start height.
        scan_title_pub (Publisher): Publisher for scan title.
        scan_ds_pub (Publisher): Publisher for scan dataset.
        ready_pub (Publisher): Publisher for ready status.
        kill_pub (Publisher): Publisher for kill signal.
        will_print_pub (Publisher): Publisher for print status.
        scan_start_sub (Subscription): Subscription for scan start signal.
        scan_stop_sub (Subscription): Subscription for scan stop signal.
        qos_profile (QoSProfile): Quality of Service profile for subscriptions.
        camera_thread (Thread): Thread for updating the camera stream.
        format: Image format for QVIO callback.
        qimage: QImage object for QVIO callback.
        pixmap: QPixmap object for QVIO callback.
    """

    def __init__(self, qvio_widget:QWidget, image_exporter, scroll_container, mlui) -> None:
        super().__init__("gui_node")
        self.qvio_widget = qvio_widget
        self.image_exporter = image_exporter
        self.scroll_container = scroll_container
        self.mlui = mlui
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
        self.scan_ds_pub = self.create_publisher(
            String, 
            "/host/gui/out/scan_ds", 
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
        self.scan_start_sub = self.create_subscription(
            Bool, 
            "/starling/out/fc/scan_start", 
            self.scan_start_callback, 
            qos_profile_system_default
        )
        self.scan_stop_sub = self.create_subscription( 
            Bool, 
            "/starling/out/fc/scan_end", 
            self.scan_stop_callback, 
            qos_profile_system_default
        )
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # self.qvio_sub = self.create_subscription(
        #     Image, 
        #     "/qvio_overlay", 
        #     self.qvio_callback, 
        #     qos_profile=qos_profile_sensor_data
        # )
        self.start_camera_thread()

    def scan_start_callback(self, msg):
        if msg.data:
            recording_circle.set_color(DTC_RED)
            self.info("Scan started")
            self.image_exporter.set_dump_status(True)
            

    def scan_stop_callback(self, msg):
        if msg.data:
            recording_circle.set_color(DTC_BLUE)
            self.info("Scan stopped")
            print("SETTING DATASET: " + self.image_exporter.get_dataset())
            self.image_exporter.set_dump_status(False)
            # self.mlui = MLUI(self.scroll_container, self.image_exporter.get_dataset(), self.image_exporter.get_class())
            print("SETTING DATASET: " + self.image_exporter.get_dataset())
            # self.mlui.set_model_name(self.image_exporter.get_class())
            # self.mlui.set_dataset(self.image_exporter.get_dataset())
            self.mlui.with_new(self.image_exporter.get_dataset(), self.image_exporter.get_class())
            self.mlui.renew()
            self.scroll_container.next()

    def start_camera_thread(self):
        self.camera_thread = threading.Thread(target=self.update_stream)
        self.camera_thread.start()

    def update_stream(self):
        while True:
            self.qvio_widget.update_frame()
            time.sleep(0.03)

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
        if self.mlui.set_model_name(title):
            self.get_logger().info("Model name set to: " + title)
        else:
            self.get_logger().info("Failed to set model name")
        self.scan_title_pub.publish(self.create_string(title))
        self.info("Publishing scan title: " + str(title))

    def publish_scan_ds(self, ds):
        if self.mlui.set_dataset(ds):
            self.get_logger().info("Dataset set to: " + ds)
        else:
            self.get_logger().info("Failed to set dataset")
        self.scan_ds_pub.publish(self.create_string(ds))
        self.info("Publishing scan dataset: " + str(ds))
    
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


class CircleWidget(QWidget):
    """
    A custom QWidget that displays a colored circle with text centered inside it.
    Attributes:
        text (str): The text to be displayed in the center of the circle.
        color (QColor): The color of the circle.
    """

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setMinimumSize(200, 100)
        self.setMaximumSize(200, 100)
        self.text = ""
        self.color = Qt.gray

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.NoPen)
        painter.setBrush(self.color)
        painter.drawRoundedRect(0, 0, self.width(), self.height(), 25, 25)

        font = painter.font()
        font.setPointSize(12)
        painter.setFont(font)
        painter.setPen(Qt.black)
        painter.drawText(event.rect(), Qt.AlignCenter, self.text)

    def set_size(self, width, height):
        self.setMinimumSize(width, height)
        self.setMaximumSize(width, height)

    def set_text(self, text):
        self.text = text
        self.update()

    def set_color(self, color):
        self.color = color
        self.update()

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

    def start_ros():
        rclpy.init(args=None)


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = FlightWidget()

    # Show the main window
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()