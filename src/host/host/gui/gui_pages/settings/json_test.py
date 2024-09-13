import json
import os

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
json_path = os.path.join(truncated_path, 'settings.json')

# Open the JSON file and load its contents
with open(json_path, "r") as json_file:
    settings = json.load(json_file)

# Access the data from the JSON file
# For example, if your JSON file contains a list of names:
for setting in settings["settings"]:
    print(setting["display_name"])