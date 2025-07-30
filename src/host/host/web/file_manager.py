import os

class FileManager:
    def __init__(self, file_directory='/var/lib/aeroprint'):
        self.file_directory = file_directory
        self.current_id = self.get_latest_id()

    def get_latest_id(self):
        """
        Get the latest ID based on existing folders in the file directory.
        This assumes that folder names are numeric and represent IDs.
        """
        folders = [
            int(name) for name in os.listdir(self.file_directory)
            if os.path.isdir(os.path.join(self.file_directory, name)) and name.isdigit()
        ]
        return max(folders) if folders else 0

    def generate_new_id(self):
        return self.get_latest_id() + 1
    
    def create_new_folder(self):
        """
        Create a new folder with the next available ID.
        """
        new_id = self.generate_new_id()
        new_folder_path = self.get_id_path(new_id)
        self.current_id = new_id  # Update current ID
        os.makedirs(new_folder_path, exist_ok=True)
        self.mkdir_pcd(new_id)  # Create pcd subfolder
        self.mkdir_images(new_id)
        self.mkdir_logs(new_id)
        return new_folder_path
    
    def get_id(self):
        """
        Get the current ID.
        """
        return self.current_id
    
    def get_id_path(self, id):
        """
        Get the path for a folder with the specified ID.
        """
        return os.path.join(self.file_directory, str(id))
    
    def mkdir_pcd(self, id):
        """
        Create a new folder for point cloud data with the specified ID.
        """
        new_folder_path = self.get_pcd_folder(id)  # Ensure the folder exists
        os.makedirs(new_folder_path, exist_ok=True)
        return new_folder_path
    
    def mkdir_images(self, id):
        """
        Create a new folder for images with the specified ID.
        """
        new_folder_path = self.get_images_folder(id)  # Ensure the folder exists
        os.makedirs(new_folder_path, exist_ok=True)
        return new_folder_path
    
    def mkdir_logs(self, id):
        """
        Create a new folder for logs with the specified ID.
        """
        new_folder_path = self.get_logs_folder(id)  # Ensure the folder exists
        os.makedirs(new_folder_path, exist_ok=True)
        return new_folder_path
    
    def get_pcd_folder(self, id):
        """
        Get the path for the point cloud data folder of the specified ID.
        """
        return os.path.join(self.get_id_path(id), 'pcd')
    
    def get_images_folder(self, id):
        """
        Get the path for the images folder of the specified ID.
        """
        return os.path.join(self.get_id_path(id), 'images')
    
    def get_logs_folder(self, id):
        """
        Get the path for the logs folder of the specified ID.
        """
        return os.path.join(self.get_id_path(id), 'logs')
    
    def write_pcd_file(self, filename, data, id=None):
        """
        Write point cloud data to a file in the specified ID's pcd folder.
        """
        if id is None:
            id = self.get_id()
        pcd_dir = self.get_pcd_folder(id)
        file_path = os.path.join(pcd_dir, filename)
        with open(file_path, 'w') as f:
            f.write(data)
        return file_path

if __name__ == "__main__":
    fm = FileManager()
    new_id = fm.generate_new_id()
    print(f"New ID generated: {new_id}")
    # This will print the next available ID based on existing folders in the specified directory.

    new_folder = fm.create_new_folder()
    print(f"New folder created: {new_folder}")
