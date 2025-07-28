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
    """
    A widget for displaying and managing application settings.

    Attributes:

        layout (QVBoxLayout): The main layout of the widget.
        settings (dict): A dictionary to store the settings loaded from a JSON file.
        json_path (str): The path to the settings JSON file.
    """

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
        """
        Load settings from a JSON file located in the 'aeroprint' directory.
        This method constructs the path to the 'settings.json' file by finding the
        'aeroprint' directory in the current file's path. It then attempts to load
        the settings from this JSON file and assigns them to the 'settings' attribute.
        
        Raises:

            ValueError: If the 'aeroprint' directory is not found in the path.
            FileNotFoundError: If the 'settings.json' file is not found.
        
        Displays:

            QMessageBox.warning: If the 'settings.json' file is not found.
        """

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
        """
        Update the setting with the given key to the specified value.
        
        Args:

            key (str): The key of the setting to update.
            value (Any): The new value for the setting.
        """

        self.settings[key] = value

    def save_settings(self):
        """
        Save the current settings to a JSON file.
        This method iterates through all widgets in the layout, checks if they are instances of SettingWidget,
        and calls their save_value method. It then attempts to write the settings to a file named 'settings.json'.
        If the operation is successful, a success message is displayed. If an error occurs, an error message is shown.
        
        Raises:
            
            Exception: If there is an error while saving the settings to the file.
        """

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
    """
    A widget for displaying and editing a setting.

    Attributes:

        setting (dict): A dictionary containing the setting details.
        display_name (str): The display name of the setting.
        description (str): The description of the setting.
        default_value (any): The default value of the setting.
        value (any): The current value of the setting.
        type (str): The type of the setting (e.g., "text", "number", "boolean", "file_path").
        layout (QVBoxLayout): The layout of the widget.
        name_widget (QLabel): The label displaying the setting's name.
        edit_widget (QWidget): The widget for editing the setting's value.
        description_widget (QLabel): The label displaying the setting's description.
    
    """

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
        """
        Creates and returns a widget for editing based on the type of the setting.
        
        Returns:
            
            QWidget: A widget appropriate for the type of the setting.
            
            - QLineEdit: If the type is "text" or "number".
            - QCheckBox: If the type is "boolean".
            - FilePickerWidget: If the type is "file_path".
        """
        
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
        """
        A custom widget that allows the user to select a folder from the file system.
        
        Attributes:

            layout (QHBoxLayout): The horizontal box layout for the widget.
            file_path_widget (QLineEdit): The line edit widget to display the selected folder path.
            file_picker_button (QPushButton): The button to open the file dialog for folder selection.
        """

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
            """
            Opens a file dialog for the user to select a folder. If a folder is selected,
            the file path is set to the text of the file_path_widget.
            
            Returns:
               
                None
            """

            file_dialog = QFileDialog()
            file_path = file_dialog.getExistingDirectory(self, "Select Folder")
            if file_path:
                self.file_path_widget.setText(file_path)

        def set_text(self, text):
            """
            Sets the text of the file_path_widget.
            
            Args:
                
                text (str): The text to set in the file_path_widget.
            """

            self.file_path_widget.setText(text)

        def text(self):
            """
            Retrieve the text from the file path widget.
            
            Returns:
                
                str: The text from the file path widget.
            """

            return self.file_path_widget.text()
        
    def save_value(self):
        """
        Save the value from the edit widget to the setting dictionary based on the type.
        The method updates the 'value' key in the 'setting' dictionary with the current value
        from the 'edit_widget'. The type of the value is determined by the 'type' attribute.
        
        Supported types:

        - "text": Saves the text from the edit widget.
        - "number": Converts the text from the edit widget to an integer and saves it.
        - "boolean": Saves the checked state of the edit widget.
        - "file_path": Saves the text from the edit widget as a file path.
        """

        if self.type == "text":
            self.setting["value"] =  self.edit_widget.text()
        elif self.type == "number":
            self.setting["value"] = int(self.edit_widget.text())
        elif self.type == "boolean":
            self.setting["value"] = self.edit_widget.isChecked()
        elif self.type == "file_path":
            self.setting["value"] = self.edit_widget.text()

    def get_setting(self):
        """
        Retrieve the current setting.
        
        Returns:

            The current setting.
        """

        return self.setting


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = SettingsWidget()
    widget.show()
    sys.exit(app.exec_())