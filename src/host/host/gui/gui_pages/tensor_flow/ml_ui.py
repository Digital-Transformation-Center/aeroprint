from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QApplication, QStackedWidget, QGridLayout, QScrollArea, QSizePolicy, QMessageBox, QLineEdit
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtCore import Qt
import sys
import os
from model_tester import ModelTester
from model_generator import ModelGenerator
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from resources.settings_utility import SettingsUtility
import cv2
import numpy as np
class MLUI(QWidget):
    def __init__(self, scroll_container, dataset, model_name):
        super().__init__()
        self.setMinimumSize(1200, 800)
        self.settings_utility = SettingsUtility()
        self.resource_path = os.path.abspath(self.settings_utility.get_value("resources_file_path"))
        self.sc = scroll_container

        self.dataset = dataset
        self.model_name = model_name
        self.image_directory = os.path.join(self.resource_path, "datasets", self.dataset, "images", self.model_name)
        self.classes_directory = os.path.join(self.resource_path, "datasets", self.dataset, "images")
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
        self.model_directory = os.path.join(self.resource_path, "datasets", self.dataset, "model", "model.keras")
        self.model_tester = ModelTester(self.model_directory)
        folder_count = len(os.listdir(self.image_directory))

        self.model_generator = ModelGenerator(folder_count, self.image_directory, self.model_directory)

        self.image_scroll_widget = ImageScrollWidget(self.settings_utility, self.dataset, self.model_name, self)
        self.model_interaction_widget = ModelInteractionWidget(self.model_generator)
        self.model_interaction_widget.show_model_prediction("None", "0", False)
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

    def image_select(self, image_path):
        if self.model_exists():

            folder_list = os.listdir(self.classes_directory)
            folder_list.reverse()

            print (folder_list)
            def load_image(image_path):
                image = cv2.imread(image_path)
                image = cv2.resize(image, (224, 224))
                if image is not None:
                    image_array = np.array(image)
                    return image_array
                else:
                    print("Failed to load image")
                    return None

            image_array = load_image(image_path)
            if image_array is not None:
                # Use the image_array for further processing
                print("Image loaded successfully")
            else:
                print("Image loading failed")
            print(image_array.shape)
            img_batch = np.expand_dims(image_array, axis=0)
            prediction, certainty = self.model_tester.test(img_batch)
            print(folder_list[prediction.argmax()], certainty)
            self.model_interaction_widget.show_model_prediction(folder_list[prediction.argmax()], str(certainty), self.model_exists())
        else:
            message = "No model exists for the selected dataset. Please generate a model for the dataset."
            QMessageBox.warning(self, "Model Not Found", message)

    def model_exists(self):
        return os.path.exists(self.model_directory)

    def set_dataset(self, dataset):
        self.dataset = dataset

    def update_object_scan(self, scan_directory):
        self.scan_directory = scan_directory
        self.image_directory = os.path.join(self.resource_path, self.dataset, "images")

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
    def __init__(self, settings_utility, dataset, model_name, parent_widget):
        super().__init__()
        self.parent_widget = parent_widget
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
            thumbnail_label.mousePressEvent = lambda event, path=image_path: self.parent_widget.image_select(path)
            self.grid_layout.addWidget(thumbnail_label, row, col)
            col += 1
            if col == 5:
                row += 1
                col = 0
            
class ModelInteractionWidget(QWidget):
    def __init__(self, model_generator):
        super().__init__()
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.label = QLabel("Select an image to interact with the model.")
        self.layout.addWidget(self.label)
        self.train_model_label = QLabel("Train the model with these images")
        self.model_generator = model_generator
        self.model_train_button = QPushButton("Train Model")
        self.model_train_button.clicked.connect(lambda: self.show_training_popup())
        self.layout.addWidget(self.train_model_label)
        self.layout.addWidget(self.model_train_button)
        self.train_model_label.hide()
        self.model_train_button.hide()
        self.next_button = QPushButton("Next")
        self.layout.addWidget(self.next_button)

    def show_model_prediction(self, prediction, certainty, model_exists):
        if model_exists:
            self.label.show()
            self.label.setText("This image most closely matches a " + prediction + ".")
        else:
            self.label.hide()
        self.model_train_button.show()
        self.train_model_label.show()

    def show_training_popup(self):
        popup = QMessageBox()
        popup.setWindowTitle("Training Model")
        popup.setText("Enter training parameters:")
        popup.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        textbox = QLineEdit()
        popup.layout().addWidget(textbox)
        button = popup.button(QMessageBox.Ok)
        button.clicked.connect(lambda: print(textbox.text()))
        popup.exec_()

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
    model_name = "lime"
    widget = MLUI(sc, dataset, model_name)
    sc.add_page(widget)
    sc.show()
    sys.exit(app.exec_())