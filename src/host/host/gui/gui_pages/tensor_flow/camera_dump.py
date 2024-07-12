import sys
import cv2
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QListWidget, QGridLayout, QHBoxLayout
import os
from gui_pages.tensor_flow.model_generator import ModelGenerator
from gui_pages.tensor_flow.model_tester import ModelTester
import numpy as np
import json
import threading
from PIL import Image
class CameraDumpGUI(QWidget):
    def __init__(self):
        super().__init__()
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
        self.path = os.path.join(truncated_path, 'src/data_management/data_management/datasets/')

        self.setWindowTitle("Teachable Machine")
        self.textbox = QLineEdit()
        self.start_button = QPushButton("Record Class")
        self.stop_button = QPushButton("Stop Recording")

        self.button_layout = QVBoxLayout()
        self.button_layout.addWidget(self.start_button)
        self.button_layout.addWidget(self.stop_button)
        self.button_widget = QWidget()
        self.button_widget.setLayout(self.button_layout)

        self.record_icon = QWidget()
        self.record_icon_recording_style = """
            background-color: red;
            border-radius: 25px;
            border: 2px solid black;
        """
        self.record_icon_paused_style = """
            background-color: grey;
            border-radius: 25px;
            border: 2px solid black;
        """
        self.record_icon.setStyleSheet(self.record_icon_paused_style)
        self.record_icon.setFixedSize(50, 50)

        self.record_layout = QHBoxLayout()
        self.record_layout.addWidget(self.button_widget)
        self.record_layout.addWidget(self.record_icon)
        self.recording_widget = QWidget()
        self.recording_widget.setLayout(self.record_layout)


        self.camera_label = QLabel()
        self.camera = cv2.VideoCapture(0)
        self.start_button.clicked.connect(self.start_dump)
        self.stop_button.clicked.connect(self.stop_dump)

        cam_stack_layout = QVBoxLayout()

        cam_stack_layout.addWidget(self.textbox)
        cam_stack_layout.addWidget(self.recording_widget)
        cam_stack_layout.addWidget(self.camera_label)

        main_layout = QHBoxLayout()
        main_layout.setSpacing(0)

        self.folder_widget = FolderListWidget(self.path)
        self.folder_widget.folder_picked_signal.connect(self.folder_picked)

        self.action_widget = ActionsWidget()


        main_layout.addWidget(self.folder_widget)
        main_layout.addLayout(cam_stack_layout)
        main_layout.addWidget(self.action_widget)
        


        self.setLayout(main_layout)
        self.image_num = 0
        self.do_dump = False
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.display_camera)
        self.camera_timer.start(1000 // 10)

    def folder_picked(self, item):
        self.dataset_path = os.path.join(self.path, item)
        self.dataset_images_path = os.path.join(self.dataset_path, "images")
        self.dataset_model_path = os.path.join(self.dataset_path, "model")
        self.dataset_resources_path = os.path.join(self.dataset_path, "resources")
        self.dataset_properties_json_path = os.path.join(self.dataset_resources_path, "properties.json")
        self.action_widget.set_images_path(self.dataset_images_path)
        self.action_widget.set_model_path(self.dataset_model_path)
        self.model_path = os.path.join(self.dataset_model_path, "model.keras")
        try:
            self.action_widget.set_test_model(ModelTester(self.model_path))
        except: pass

    def display_camera(self):
        ret, frame = self.camera.read()
        if ret:
            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self.action_widget.set_current_frame(bgr_frame)
            scaled_frame = cv2.resize(frame, (frame.shape[1]//2, frame.shape[0]//2))
            image = QImage(scaled_frame.data, scaled_frame.shape[1], scaled_frame.shape[0], QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(image)
            self.camera_label.setPixmap(pixmap)
            if self.do_dump:
                self.save_frame(bgr_frame)

    def start_dump(self):
        directory = os.path.join(self.dataset_images_path, self.textbox.text())
        if not os.path.exists(directory):
            os.makedirs(directory)
        self.dump_dir = directory
        self.do_dump = True
        self.record_icon.setStyleSheet(self.record_icon_recording_style)

    def stop_dump(self):
        self.do_dump = False
        self.record_icon.setStyleSheet(self.record_icon_paused_style)
    
    def save_frame(self, frame):
        image_path = os.path.join(self.dump_dir, str(self.image_num) + ".jpeg")
        cv2.imwrite(image_path, frame)
        self.image_num += 1

class FolderListWidget(QWidget):
    folder_picked_signal = pyqtSignal(str)

    def __init__(self, directory):
        super().__init__()
        self.directory = directory
        self.setWindowTitle("Dataset List")
        self.textbox = QLineEdit()
        self.add_button = QPushButton("Create Dataset")
        self.folder_list = QListWidget()
        self.folder_list.setMinimumWidth(200)
        self.populate_folder_list()
        self.add_button.clicked.connect(self.add_folder)
        layout = QVBoxLayout()
        layout.addWidget(self.textbox)
        layout.addWidget(self.add_button)
        layout.addWidget(self.folder_list)
        self.folder_list.itemClicked.connect(self.print_selected_folder)
        self.setLayout(layout)

    def print_selected_folder(self, item):
        self.folder_picked_signal.emit(item.text())

    def populate_folder_list(self):
        folders = os.listdir(self.directory)
        self.folder_list.clear()
        self.folder_list.addItems(folders)

    def add_folder(self):
        folder_name = self.textbox.text()
        if folder_name:
            folder_path = os.path.join(self.directory, folder_name)
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)
                images_path = os.path.join(folder_path, "images")
                model_path = os.path.join(folder_path, "model")
                resources_path = os.path.join(folder_path, "resources")
                properties_json_path = os.path.join(resources_path, "properties.json")
                os.makedirs(images_path)
                os.makedirs(model_path)
                os.makedirs(resources_path)
                with open(properties_json_path, "w") as f:
                    f.write("{}")
                self.populate_folder_list()
            else:
                print("Folder already exists.")
        else:
            print("Please enter a folder name.")

class ActionsWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Actions")
        self.train_button = QPushButton("Train Model")
        self.train_button.clicked.connect(self.train_model)
        self.test_button = QPushButton("Test Model")
        self.test_button.clicked.connect(self.test_model)
        self.prediction_label = QLabel("Prediction")
        self.prediction_text = QLabel()
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.training_box = QLabel()
        self.training_box.setStyleSheet("background-color: green;")
        self.training_box.setFixedSize(100, 25)
        layout.addWidget(self.train_button)
        layout.addWidget(self.training_box)
        layout.addWidget(self.test_button)
        layout.addWidget(self.prediction_label)
        layout.addWidget(self.prediction_text)
        self.setLayout(layout)

    def train_model(self):
        folder_count = len(os.listdir(self.images_path))

        gen = ModelGenerator(folder_count, self.images_path, self.model_path)
        def generate_model():
            self.training_box.setStyleSheet("background-color: red;")
            gen.generate()
            self.training_box.setStyleSheet("background-color: green;")

        thread = threading.Thread(target=generate_model)
        thread.start()

    def test_model(self):
        model_path = os.path.join(self.model_path, "model.keras")
        
        # Convert cv2 image to PIL Image
        pil_image = Image.fromarray(cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB))
        pil_image = pil_image.resize((224, 224))
        img = np.asarray(pil_image)
        img_batch = np.expand_dims(img, axis=0)
        # Get the list of folders in the images path
        folder_list = os.listdir(self.images_path)
        folder_list.reverse()
        print(img.shape)
        prediction, certainty = self.model_tester.test(img_batch)
        if (certainty < 0.9):
            self.prediction_text.setText("Unknown")
        else:
            self.prediction_text.setText(folder_list[prediction.argmax()])
        print(folder_list[prediction.argmax()], certainty)


    def set_images_path(self, images_path):
        self.images_path = images_path
    
    def set_model_path(self, model_path):
        self.model_path = model_path

    def set_current_frame(self, frame):
        self.current_frame = frame

    def set_test_model(self, model_tester):
        self.model_tester = model_tester

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('fusion')
    gui = CameraDumpGUI()
    gui.show()
    sys.exit(app.exec())