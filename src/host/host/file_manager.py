from host.gui.resources.settings_utility import SettingsUtility
import os
class FileManager:
    def __init__(self):
        self.settings_utility = SettingsUtility()
        self.resources_path = self.settings_utility.get_value('resources_file_path')
        self.datasets_path = os.path.join(self.resources_path, 'datasets')
        self.class_name = ""
        self.dataset_name = ""
    
    def set_class(self, class_name):
        self.class_name = class_name
        self.class_path = os.path.join(self.datasets_path, self.dataset_name, class_name)
        self.images_path = os.path.join(self.class_path, 'images/')
        self.model_path = os.path.join(self.class_path, 'model/')
        self.pointclouds_path = os.path.join(self.class_path, 'pointclouds/')
        self.meshes_path = os.path.join(self.class_path, 'meshes/')

    def set_dataset(self, dataset_name):
        self.dataset_name = dataset_name
        self.set_class(self.class_name)

    def get_pointclouds_path(self):
        return self.pointclouds_path
    