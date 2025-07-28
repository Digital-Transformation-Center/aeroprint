from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QApplication, QStackedWidget, QGridLayout, QScrollArea, QSizePolicy, QMessageBox, QLineEdit, QComboBox, QProgressBar
from PyQt5.QtGui import QPixmap, QIcon, QMovie
from PyQt5.QtCore import Qt, QSize, QRect, QThread, QTimer, QObject, pyqtSignal
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))

from host.gui.gui_pages.tensor_flow.model_tester import ModelTester
from host.gui.gui_pages.tensor_flow.model_generator import ModelGenerator
from host.gui.resources.settings_utility import SettingsUtility
import cv2
import numpy as np
import random
import shutil
import threading

class MLUI(QWidget):
    def __init__(self, scroll_container, dataset, model_name):
        super().__init__()
        self.signals = WorkerSignals()
        self.signals.finished.connect(self.on_load_ui_assets)
        self.setMinimumSize(1200, 800)
        self.settings_utility = SettingsUtility()
        self.resource_path = os.path.abspath(self.settings_utility.get_value("resources_file_path"))
        self.sc = scroll_container

        self.dataset = dataset
        self.model_name = model_name
        self.set_image_directory()
        self.set_classes_directory()
        self.set_model_directory()
        layout = QVBoxLayout()
        self.splash_widget = SplashWidget()
        self.widget_stack = QStackedWidget()
        self.widget_stack.addWidget(self.splash_widget)
        layout.addWidget(self.widget_stack)
        self.splash_widget.next_button.clicked.connect(self.load_ui)
        self.setLayout(layout)
        self.new_mit()

    def set_model_name(self, model_name):
        self.with_new(self.dataset, model_name)
        # self.model_name = model_name
        # self.set_image_directory()
        return True

    def set_dataset(self, dataset): 
        self.with_new(dataset, self.model_name)
        # self.dataset = dataset
        # self.set_image_directory()
        # self.set_classes_directory()
        # self.set_model_directory()
        # self.new_mit()
        return True

    def with_new(self, dataset, model_name):
        self.dataset = dataset
        self.model_name = model_name
        self.set_image_directory()
        self.set_classes_directory()
        self.set_model_directory()
        self.new_mit()


    def set_image_directory(self):
        self.image_directory = os.path.join(self.resource_path, "datasets", self.dataset, self.model_name, "images/")

    def set_classes_directory(self):
        print("SETTING CLASSES DIRECTORY: " + self.dataset)
        self.classes_directory = os.path.join(self.resource_path, "datasets", self.dataset)

    def set_model_directory(self):
        self.model_directory = os.path.join(self.resource_path, "datasets", self.dataset, "model", "model.keras")

    def new_mit(self):
        self.mit = ModelInteractionToolkit(self.classes_directory, self.image_directory, None, None, self.model_directory)

    def renew(self):
        self.widget_stack.setCurrentIndex(0)
        self.splash_widget = SplashWidget()
        self.widget_stack = QStackedWidget()
        self.widget_stack.addWidget(self.splash_widget)
        self.splash_widget.next_button.clicked.connect(self.load_ui)

    def load_ui(self):
        
        load_ui_thread = threading.Thread(target=self.load_ui_assets)
        load_ui_thread.start()
        self.present_ui_loader()
        print("Assets loaded")
        
    def on_load_ui_assets(self):
        self.image_scroll_widget = ImageScrollWidget(self.settings_utility, self.dataset, self.model_name, self)
        model_path = os.path.join(self.model_export_directory, "model.keras")
        self.model_interaction_widget = ModelInteractionWidget(self.model_generator, self.model_tester, self.folder_list, self.classes_directory, self.model_name, model_path, self.sc.next_button())
        self.model_interaction_widget.set_mit(self.mit)
        if self.model_exists():
            self.model_interaction_widget.set_prediction(self.test_set_prediction)
        self.model_interaction_widget.show_model_prediction("None", "0", False)
        self.scan_output_widget = ScanOutputWidget(self.image_directory, self.model_directory, self.dataset)
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
        self.lp.close_popup()
        
    def present_ui_loader(self):
        self.lp = LoadingPopup(10) # 6 seconds loading time
        self.lp.show_popup()

    def load_ui_assets(self):
        self.model_directory = os.path.join(self.resource_path, "datasets", self.dataset, "model", "model.keras")
        self.model_export_directory = os.path.join(self.resource_path, "datasets", self.dataset, "model/")
        self.model_tester = ModelTester()
        self.mit.assign_model_tester(self.model_tester)
        if self.model_exists():
            self.test_set_prediction = self.mit.get_best_match()
            print(self.test_set_prediction)
        folder_count = len(os.listdir(self.image_directory))
        self.model_generator = ModelGenerator(folder_count, self.image_directory, self.model_directory)
        self.folder_list = os.listdir(self.classes_directory)
        self.folder_list.reverse()
        self.signals.finished.emit()

    def image_select(self, image_path):
        if self.model_exists():
            print (self.folder_list)
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
            print(self.folder_list[prediction.argmax()], certainty)
            self.model_interaction_widget.show_model_prediction(self.folder_list[prediction.argmax()], str(certainty), self.model_exists())
        else:
            message = "No model exists for the selected dataset. Please generate a model for the dataset."
            QMessageBox.warning(self, "Model Not Found", message)

    def model_exists(self):
        model_path = os.path.join(self.model_directory, "model.keras")
        return os.path.exists(model_path)

    def set_dataset(self, dataset):
        self.dataset = dataset

    def update_object_scan(self, scan_directory):
        self.scan_directory = scan_directory
        self.image_directory = os.path.join(self.resource_path, self.dataset, "images/")

class SplashWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.label = QLabel("Welcome to AeroPrint AI\nPlease click Continue to continue.")
        self.label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label)
        self.next_button = QPushButton("Continue")
        self.layout.addWidget(self.next_button)

class ImageScrollWidget(QWidget):
    def __init__(self, settings_utility, dataset, model_name, parent_widget):
        super().__init__()
        self.parent_widget = parent_widget
        self.settings_utility = settings_utility
        self.dataset = dataset
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
        image_directory = os.path.join(self.settings_utility.get_value("resources_file_path"), "datasets", self.dataset, self.model_name, "images")
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
    def __init__(self, model_generator, model_tester, folder_list, classes_directory, class_name, model_path, next_button=None):
        super().__init__()
        self.prediction = None
        self.best_match = None
        self.prediction_signal = pyqtSignal()
        self.prediction_signal.connect(self.load_model_popup)
        self.model_path = model_path
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.label = QLabel("Select an image to interact with the model.")
        self.layout.addWidget(self.label)
        self.train_model_label = QLabel("Train the model with these images")
        self.model_generator = model_generator
        self.folder_list = folder_list
        self.classes_directory = classes_directory
        self.images_directory = os.path.join(self.classes_directory, class_name)
        self.model_tester = model_tester
        self.model_train_button = QPushButton("Train Model")
        self.model_train_button.clicked.connect(lambda: self.show_training_popup())
        self.layout.addWidget(self.train_model_label)
        self.layout.addWidget(self.model_train_button)
        self.train_model_label.hide()
        self.model_train_button.hide()
        # Change this to be fed a next button from the parent widget
        self.next_button = next_button
        self.layout.addWidget(self.next_button)
        self.mit = ModelInteractionToolkit(self.classes_directory, self.images_directory, self.model_tester, self.prediction_signal, self.model_path)
        self.model_exists = self.mit.model_exists()
        self.lp = None

    def set_mit(self, mit):
        self.mit = mit
        
    def set_next_button(self, next_button):
        self.next_button = next_button

    def set_prediction(self, prediction):
        self.prediction = prediction

    def show_model_prediction(self, prediction, certainty, model_exists):
        if model_exists:
            self.label.show()
            self.label.setText("This image most closely matches a " + prediction + ".")
        else:
            self.label.hide()
        self.model_train_button.show()
        self.train_model_label.show()
    
    def show_training_popup(self):
        if self.prediction is not None:
            self.best_match = self.prediction
            self.load_model_popup()
        else:
            print("MODEL PATH: " + self.model_path)
            self.model_exists = os.path.exists(self.model_path)
            if self.model_exists:
                gbm_thread = threading.Thread(target=self.mit.get_best_match)
                gbm_thread.start()
                self.present_prediction_loader()
            else:
                self.load_model_popup()
            
        

    def load_model_popup(self):
        popup = QMessageBox()
        popup.setWindowTitle("Training Model")
        training_widget = QWidget()
        training_layout = QVBoxLayout(training_widget)

        # popup.setText("Enter training parameters:") 
        description_text = QLabel("Train the model by creating a new class,\nusing the predicted class, or type an\nexisting class name.")
        if self.model_exists:
            pred_text = QLabel(f"Model prediction: {self.mit.best_match}")
            options = ["New Class", f"Model Prediction: {self.mit.best_match}"]
            # Append options with the name of every directory in classes directory except the predicted one
            # self.folder_list.remove(self.mit.best_match)
            options.extend(self.folder_list)
            options.remove(self.mit.best_match)

        else:
            pred_text = QLabel("No model exists")
            options = ["New Class"]
            options.extend(self.folder_list)
            
        selector = QComboBox()
        selector.addItems(options)
        selector.currentIndexChanged.connect(self.on_selector_changed)
        training_layout.addWidget(description_text)
        training_layout.addWidget(selector)
        training_layout.addWidget(pred_text)
        self.model_name_textbox = QLineEdit()
        
        training_layout.addWidget(self.model_name_textbox)
        popup.layout().addWidget(training_widget)
        popup.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        
        button = popup.button(QMessageBox.Ok)
        button.clicked.connect(lambda: self.train_model(selector.currentText(), self.model_name_textbox.text(), self.best_match))
        if self.lp is not None:
            self.lp.close_popup()
        popup.exec_()

    def on_selector_changed(self, index):
        if index == 0:
            self.model_name_textbox.show()
        else:
            self.model_name_textbox.hide()

    def present_prediction_loader(self):
        self.lp = LoadingPopup(6) # 6 seconds loading time
        self.lp.show_popup()


    def train_model(self, selection, custom_text, predicted_name):
        self.new_model_name = ""
        if selection == "New Class":
            self.new_model_name = custom_text
        elif selection == f"Model Prediction: {predicted_name}":
            self.new_model_name = predicted_name
        else:
            self.new_model_name = selection
        # Move all files from source directory to destination directory
        source_directory = self.images_directory
        destination_directory = os.path.join(self.classes_directory, self.new_model_name)
        shutil.move(source_directory, destination_directory)
        self.folder_list = os.listdir(self.classes_directory)
        if not os.path.exists(self.model_path):
            os.makedirs(self.model_path)
        gen = ModelGenerator(len(self.folder_list), self.classes_directory, self.model_path)
        def generate_model():
            gen.generate()
            self.show_training_complete_popup()

        thread = threading.Thread(target=generate_model)
        thread.start()
        popup = QMessageBox()
        popup.setWindowTitle("Training Model")
        popup.setText("""
                      You will be alerted when training is complete. Do not close this application.
                      \nThis may take a few minutes. Larger image sets will require more time.""")
        popup.setStandardButtons(QMessageBox.Ok)
        popup.exec_()

    def show_training_complete_popup(self):
        popup = QMessageBox()
        popup.setWindowTitle("Model Trained")
        popup.setText("Your new image recognition model is complete and ready to use.")
        popup.setStandardButtons(QMessageBox.Ok)
        self.images_directory = os.path.join(self.classes_directory, self.new_model_name)
        self.model_path = os.path.join(self.model_path, "model.keras")
        self.mit = ModelInteractionToolkit(self.classes_directory, self.images_directory, self.model_tester, self.prediction_signal, self.model_path)
        self.prediction = self.mit.get_best_match()

        popup.exec_()
        

class ScanOutputWidget(QWidget):
    def __init__(self, images_file_path, model_path, dataset):
        super().__init__()
        self.dataset_name_label = QLabel("Current Dataset: " + dataset)
        self.image_count_label = QLabel(str(len(os.listdir(images_file_path))) + " Images")
        self.model_exists_label = QLabel("Dataset Model Present")
        if not os.path.exists(model_path):
            self.model_exists_label.setText("Model not yet created")
        self.layout = QHBoxLayout()
        self.setLayout(self.layout)
        self.layout.addWidget(self.dataset_name_label)
        self.layout.addWidget(self.model_exists_label)
        self.layout.addWidget(self.image_count_label)



class LoadingPopup(QWidget):
    def __init__(self, time):
        super().__init__()
        self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setMinimumSize(600, 400)
        self.timeout_time = (time * 1000) / 100

        self.label = QLabel("Loading...")
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        
        self.popup_layout = QVBoxLayout()
        self.popup_layout.addWidget(self.label)
        self.popup_layout.addWidget(self.progress_bar)
        self.setLayout(self.popup_layout)

    def show_popup(self):
        self.show()
        self.progress_bar.setValue(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_progress)
        print(self.timeout_time)
        self.timer.start(int(self.timeout_time))

    def update_progress(self):
        current_value = self.progress_bar.value()
        if current_value < 100:
            self.progress_bar.setValue(current_value + 1)
        else:
            self.timer.stop()
            # self.close()

    def close_popup(self):
        self.close()
        # self.animation.stop()
        # self.popup.close()

class ModelInteractionToolkit():
    def __init__(self, classes_directory, images_directory, model_tester, prediction_signal, model_directory):
        self.classes_directory = classes_directory
        self.images_directory = images_directory
        self.model_tester = model_tester
        self.prediction_signal = prediction_signal
        self.model_directory = model_directory
        self.best_match = None
    # A function to get the most common prediction of all opf the images in a directory
    def get_best_match(self):
        model_path = os.path.join(self.model_directory, "model.keras")
        self.model_tester.load_model(model_path)
        best_match = None
        if self.model_exists():
            predictions = []
            folder_list = os.listdir(self.classes_directory)
            folder_list.reverse()
            print("bm_folder_list")
            print(folder_list)

            image_files = os.listdir(self.images_directory)
            random_images = random.sample(image_files, 5)
            for image_file in random_images:
                image_path = os.path.join(self.images_directory, image_file)
                print(image_path)
                image_array = cv2.imread(image_path)
                image_array = cv2.resize(image_array, (224, 224))
                img_batch = np.expand_dims(image_array, axis=0)
                prediction, _ = self.model_tester.test(img_batch)
                predictions.append(folder_list[prediction.argmax()])

            best_match = max(set(predictions), key=predictions.count)
            self.best_match = best_match
            if self.prediction_signal is not None:
                self.best_match = best_match
                self.prediction_signal.emit()
        self.best_match = best_match
        return best_match

    def assign_model_tester(self, model_tester):
        self.model_tester = model_tester

    def model_exists(self):
        model_path = os.path.join(self.model_directory, "model.keras")
        return os.path.exists(model_path)

if __name__ == "__main__":
    from scroll_container import ScrollContainer

    app = QApplication(sys.argv)
    sc = ScrollContainer()
    dataset = "fruits"
    model_name = "banana"
    widget = MLUI(sc, dataset, model_name)
    widget.set_dataset("My Dataset")
    widget.set_model_name("test 3")
    sc.add_page(widget)
    sc.add_page(QWidget())
    sc.create_page_buttons()
    sc.show()
    sys.exit(app.exec_())