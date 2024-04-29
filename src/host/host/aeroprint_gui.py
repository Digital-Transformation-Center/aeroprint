import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget, QSplashScreen, QGridLayout, QDoubleSpinBox
from PyQt5.QtGui import QPixmap, QPainter, QPen, QColor
from PyQt5.QtCore import QTimer, Qt
ENABLE_SPLASH = True
SPLASH_TIME = 3000 # 3s splash screen
DTC_BLUE = QColor(0, 189, 247)
DTC_RED = QColor(232, 37, 41)
BLACK = QColor(0, 0, 0)

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.flashSplash()
        self.layout = QGridLayout(self)
        self.parameter_widget = ParameterWidget()
        # self.height_graphic_widget = HeightGraphic()
        self.layout.addWidget(self.parameter_widget, 0, 0)
        # self.layout.addWidget(self.height_graphic_widget, 1, 0)
        # Set layout
        central_widget = QWidget()
        central_widget.setLayout(self.layout)
        # central_widget = ParameterWidget()
        self.setCentralWidget(central_widget)

    def flashSplash(self):
        self.splash = QSplashScreen(QPixmap('src/host/host/splash.png'))
        self.splash.show()
        QTimer.singleShot(SPLASH_TIME, self.splash.close)

    def show(self):
        if ENABLE_SPLASH:
            self.flashSplash()
            QTimer.singleShot(SPLASH_TIME, super().show)
        else:
            super().show()
        
class ParameterWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.scan_name_textbox = QLineEdit(self)
        self.scan_name_textbox.setMaxLength(25)
        # self.button1 = QPushButton('Print Textbox 1', self)

        # Create a QDoubleSpinBox
        self.radius_spin_box = QDoubleSpinBox(self)
        self.radius_spin_box.setDecimals(1)  # Scan radius
        self.radius_spin_box.setRange(0.7, 3.0)  # Set the allowed value range        
        self.height_spin_box = QDoubleSpinBox(self)
        self.height_spin_box.setDecimals(2)  # Scan radius
        self.height_spin_box.setRange(0.0, 5.0)  # Set the allowed value range
        self.start_height_spin_box = QDoubleSpinBox(self)
        self.start_height_spin_box.setDecimals(2)  # Scan radius
        self.start_height_spin_box.setRange(0.0, 5.0)  # Set the allowed value range
        self.scan_title = ""
        self.radius = 0.0
        self.height = 0.0
        self.start_height = 0.0
        self.start_description = QLabel('')
        self.update_start_description()
        # Connect buttons to functions
        # self.button1.clicked.connect(self.print_textbox1)
        self.radius_spin_box.valueChanged.connect(self.update_radius)
        self.height_spin_box.valueChanged.connect(self.update_height)
        self.start_height_spin_box.valueChanged.connect(self.update_start_height)

        # Create layout
        layout = QVBoxLayout()
        layout.addWidget(QLabel('Scan Title:'))
        layout.addWidget(self.scan_name_textbox)
        # layout.addWidget(self.button1)
        layout.addWidget(QLabel('Scan Radius (From Center in Meters): '))
        layout.addWidget(self.radius_spin_box)
        # layout.addWidget(self.label)
        layout.addWidget(QLabel('Object Height (Approx. in Meters): '))
        layout.addWidget(self.height_spin_box)
        layout.addWidget(QLabel('Object Starting Height (From ground in Meters): '))
        layout.addWidget(self.start_height_spin_box)
        layout.addWidget(self.start_description)

        self.setLayout(layout)

    def update_start_description(self):
        self.start_description.setText("Start scan at {r:.1f}m for an object that is approximately {h:.2f}m tall, starting at {s:.2f}m above the ground.".format(r=self.radius, h=self.height, s=self.start_height))


    def print_textbox1(self):
        print(f"Textbox 1 contents: {self.scan_name_textbox.text()}")

    def print_textbox2(self):
        self.flashSplash()
        print(f"Textbox 2 contents: {self.textbox2.text()}")

    def update_radius(self, value):
        self.radius = value
        self.update_start_description()

    def update_height(self, value):
        self.height = value
        self.update_start_description()

    def update_start_height(self, value):
        self.start_height = value
        self.update_start_description()

    def update_scan_title(self, value):
        self.scan_title = value
        self.update_start_description()

    def get_radius(self):
        return self.radius
    
    def get_height(self):
        return self.height

    def get_start_height(self):
        return self.start_height

    def get_scan_title(self):
        return self.scan_title

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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()

    # Show the main window
    window.show()
    sys.exit(app.exec_())
