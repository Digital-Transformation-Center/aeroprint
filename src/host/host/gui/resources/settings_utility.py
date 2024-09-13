import json
import os

class SettingsUtility():
    def __init__(self) -> None:
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
            print("Settings file not found.")
    
    def get_value(self, key):
        return self.settings[key]['value']