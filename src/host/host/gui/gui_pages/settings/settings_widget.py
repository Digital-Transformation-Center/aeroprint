import sys
import json
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox, QCheckBox, QHBoxLayout, QFileDialog
import os
import PyQt5.QtCore as QtCore
import sys
import json
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox, QCheckBox, QHBoxLayout, QFileDialog
import os
import PyQt5.QtCore as QtCore
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from resources.custom_widgets import CenteredButton
class SettingsWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Settings Widget")
        self.layout = QVBoxLayout()
        self.settings = {}

        # Load settings from JSON file
        self.load_settings()
        settings_widgets = []
        for setting in self.settings:
            setting_widget = SettingWidget(self.settings[setting])
            settings_widgets.append(setting_widget)
            self.layout.addWidget(setting_widget)
        save_button = CenteredButton("Save Settings")
        save_button.clicked.connect(self.save_settings)
        self.layout.addWidget(save_button)
        self.layout.setSpacing(30)
        self.layout.setAlignment(QtCore.Qt.AlignTop)
        self.setLayout(self.layout)

    def load_settings(self):
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
        self.json_path = os.path.join(truncated_path, 'settings.json')
        try:
            with open(self.json_path, "r") as file:
                self.settings = json.load(file)
        except FileNotFoundError:
            QMessageBox.warning(self, "Error", "Settings file not found.")

    def update_setting(self, key, value):
        self.settings[key] = value

    def save_settings(self):
        for i in range(self.layout.count()):
            widget = self.layout.itemAt(i).widget()
            if isinstance(widget, SettingWidget):
                widget.save_value()
        try:
            with open("settings.json", "w") as file:
                json.dump(self.settings, file, indent=4)
            QMessageBox.information(self, "Success", "Settings saved successfully.")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to save settings: {str(e)}")

class SettingWidget(QWidget):
    def __init__(self, setting):
        super().__init__()
        self.setting = setting
        self.display_name = setting["display_name"]
        self.description = setting["description"]
        self.default_value = setting["default_value"]
        self.value = setting["value"]
        self.type = setting["type"]

        self.layout = QVBoxLayout()
        self.layout.setSpacing(5)
        self.layout.setAlignment(QtCore.Qt.AlignTop)
        self.name_widget = QLabel(self.display_name)
        self.name_widget.setObjectName('name')
        self.edit_widget = self.create_edit_widget()
        self.description_widget = QLabel(self.description)
        self.description_widget.setObjectName('description')
        self.layout.addWidget(self.name_widget)
        self.layout.addWidget(self.edit_widget)
        self.layout.addWidget(self.description_widget)
        self.setLayout(self.layout)

    def create_edit_widget(self):
        if self.type == "text":
            return QLineEdit(self.value)
        elif self.type == "number":
            return QLineEdit(str(self.value))
        elif self.type == "boolean":
            checkbox = QCheckBox()
            checkbox.setChecked(self.value)
            return checkbox
        elif self.type == "file_path":
            file_path_widget = self.FilePickerWidget()
            file_path_widget.set_text(self.value)
            return file_path_widget

    class FilePickerWidget(QWidget):
        def __init__(self):
            super().__init__()
            self.layout = QHBoxLayout()
            self.file_path_widget = QLineEdit()
            self.file_picker_button = QPushButton("Select Folder")
            self.file_picker_button.clicked.connect(self.open_file_dialog)
            self.layout.addWidget(self.file_path_widget)
            self.layout.addWidget(self.file_picker_button)
            self.setLayout(self.layout)

        def open_file_dialog(self):
            file_dialog = QFileDialog()
            file_path = file_dialog.getExistingDirectory(self, "Select Folder")
            if file_path:
                self.file_path_widget.setText(file_path)

        def set_text(self, text):
            self.file_path_widget.setText(text)

        def text(self):
            return self.file_path_widget.text()
        
    def save_value(self):
        if self.type == "text":
            self.setting["value"] =  self.edit_widget.text()
        elif self.type == "number":
            self.setting["value"] = int(self.edit_widget.text())
        elif self.type == "boolean":
            self.setting["value"] = self.edit_widget.isChecked()
        elif self.type == "file_path":
            self.setting["value"] = self.edit_widget.text()

    def get_setting(self):
        return self.setting


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = SettingsWidget()
    widget.show()
    sys.exit(app.exec_())