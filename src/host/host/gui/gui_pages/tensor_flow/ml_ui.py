from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QApplication, QStackedWidget, QGridLayout, QScrollArea, QSizePolicy
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtCore import Qt
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from resources.settings_utility import SettingsUtility
class MLUI(QWidget):
    def __init__(self, scroll_container, dataset, model_name):
        super().__init__()
        self.setMinimumSize(800, 600)
        self.settings_utility = SettingsUtility()
        self.resource_path = os.path.abspath(self.settings_utility.get_value("resources_file_path"))
        self.sc = scroll_container

        self.dataset = dataset
        self.model_name = model_name

        layout = QVBoxLayout()
        self.splash_widget = SplashWidget()
        self.widget_stack = QStackedWidget()
        self.widget_stack.addWidget(self.splash_widget)
        layout.addWidget(self.widget_stack)
        self.splash_widget.next_button.clicked.connect(self.load_ui)
        self.setLayout(layout)

    def renew(self):
        self.widget_stack.setCurrentIndex(0)
        self.splash_widget = SplashWidget()
        self.widget_stack = QStackedWidget()
        self.widget_stack.addWidget(self.splash_widget)
        self.splash_widget.next_button.clicked.connect(self.load_ui)

    def load_ui(self):
        self.image_scroll_widget = ImageScrollWidget(self.settings_utility, self.dataset, self.model_name)
        self.model_interaction_widget = ModelInteractionWidget()
        self.scan_output_widget = ScanOutputWidget()
        self.ui_layout = QVBoxLayout()
        self.ui_sub_layout = QHBoxLayout()
        self.ui_sub_layout.addWidget(self.image_scroll_widget)
        self.ui_sub_layout.addWidget(self.model_interaction_widget)
        self.ui_layout.addWidget(self.scan_output_widget)
        self.ui_layout.addLayout(self.ui_sub_layout)
        self.ui_widget = QWidget()
        self.ui_widget.setLayout(self.ui_layout)
        self.widget_stack.addWidget(self.ui_widget)
        self.widget_stack.setCurrentIndex(1)


    def set_dataset(self, dataset):
        self.dataset = dataset

    def update_object_scan(self, scan_directory):
        self.scan_directory = scan_directory
        self.image_directory = os.path.join(self.resource_path, self.dataset, "images")
        self.model_directory = os.path.join(self.resource_path, self.dataset, "model", "model.keras")

class SplashWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.label = QLabel("Welcome to AeroPrint AI\nPlease click next to continue.")
        self.label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label)
        self.next_button = QPushButton("Next")
        self.layout.addWidget(self.next_button)

class ImageScrollWidget(QWidget):
    def __init__(self, settings_utility, dataset, model_name):
        super().__init__()
        self.settings_utility = settings_utility
        self.dataaset = dataset
        self.model_name = model_name
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll_widget = QWidget()
        self.grid_layout = QGridLayout()
        self.scroll_widget.setLayout(self.grid_layout)
        self.scroll_area.setWidget(self.scroll_widget)
        self.layout.addWidget(self.scroll_area)
        self.load_images()

    def load_images(self):
        image_directory = os.path.join(self.settings_utility.get_value("resources_file_path"), "datasets", self.dataaset, "images", self.model_name)
        image_files = os.listdir(image_directory)
        row = 0
        col = 0
        for image_file in image_files:
            image_path = os.path.join(image_directory, image_file)
            thumbnail_label = QLabel()
            thumbnail_label.setPixmap(QPixmap(image_path).scaled(100, 100, Qt.AspectRatioMode.KeepAspectRatio))
            thumbnail_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            thumbnail_label.mousePressEvent = lambda event, path=image_path: self.view_image(path)
            self.grid_layout.addWidget(thumbnail_label, row, col)
            col += 1
            if col == 5:
                row += 1
                col = 0
            

    def view_image(self, image_path):
        # Implement the logic to view the image when clicked
        print(image_path)
class ModelInteractionWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.label = QLabel("Model Interaction Widget")
        self.layout.addWidget(self.label)
        self.next_button = QPushButton("Next")
        self.layout.addWidget(self.next_button)

class ScanOutputWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.label = QLabel("Scan Output Widget")
        self.layout.addWidget(self.label)
        self.next_button = QPushButton("Next")
        self.layout.addWidget(self.next_button)

if __name__ == "__main__":
    from scroll_container import ScrollContainer
    app = QApplication(sys.argv)
    sc = ScrollContainer()
    dataset = "fruits"
    model_name = "banana"
    widget = MLUI(sc, dataset, model_name)
    sc.add_page(widget)
    sc.show()
    sys.exit(app.exec_())