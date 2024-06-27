import subprocess
from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QHBoxLayout, QMainWindow, QVBoxLayout, QDialog
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt, QTimer
import os
import paramiko
import threading

class MainWindow(QMainWindow):
    """
    Represents the main window of the AeroPrint application.

    This class inherits from QMainWindow and provides the GUI interface for AeroPrint.
    It contains widgets for checking the reachability of WiFi connection, Starling connection,
    and Starling startup. It also starts the GUI and runs the necessary checks.

    Attributes:
        checker (Checker): An instance of the Checker class for managing the checks.
    """

    def __init__(self):
        super().__init__()
        self.checker = Checker()
        self.setWindowTitle("AeroPrint Startup")
        self.starling_connection_widget = StarlingConnectionWidget(self.checker)
        self.wifi_connection_widget = WifiConnectionWidget(self.checker)
        self.warning = Warning("")
        self.starling_started_widget = StarlingStartedWidget(self.starling_connection_widget, self.checker)
        central_widget = QWidget()
        
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.wifi_connection_widget)
        self.layout.addWidget(self.starling_connection_widget)
        self.layout.addWidget(self.starling_started_widget)
        central_widget.setLayout(self.layout)
        self.setCentralWidget(central_widget)
        self.checker.add_check("wifi")
        self.checker.add_check("starling_connection")
        self.checker.add_check("starling_started")
        run_checks_thread = threading.Thread(target=self.run_checks)
        run_checks_thread.start()
        
        
        # self.run_checks()

    def run_checks(self):
        """
        Runs the necessary checks for WiFi connection, Starling connection, and Starling startup.

        This method is called to check the reachability of WiFi connection, Starling connection,
        and Starling startup. It calls the respective methods of the widgets and starts the GUI.
        """
        self.wifi_connection_widget.check_reachability()
        self.starling_connection_widget.check_reachability()
        self.starling_started_widget.check_reachability()
        self.start_gui()
    
    def start_gui(self):
        """
        Starts the GUI if all checks have passed.

        This method is called to start the GUI if all the checks for WiFi connection, Starling connection,
        and Starling startup have passed. Otherwise, it prints "NOT DONE" and schedules the start_gui method
        to be called again after 1 second.
        """
        if self.checker.all_checks_passed():
            launch_command = 'source ~/aeroprint/install/setup.bash && ros2 launch host host_launch.py'
            subprocess.Popen(launch_command, shell=True)
            self.close()
            print("DONE")
        else:
            print("NOT DONE")
            QTimer.singleShot(1000, self.start_gui)
            

class StarlingStartedWidget(QWidget):
    def __init__(self, connection_widget, checker):
        super().__init__()
        self.checker = checker
        self.icon_size = 50
        layout = QHBoxLayout()
        self.connection_widget = connection_widget
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
        self.text_label = QLabel(self)
        self.text_label.setText("Starling started")
        self.text_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.check_path = os.path.join(truncated_path, 'src/host/images/check.png')  
        self.x_path = os.path.join(truncated_path, 'src/host/images/x.png')  
        self.label = QLabel(self)
        self.label.setPixmap(QPixmap(self.x_path).scaled(self.icon_size, self.icon_size))
        self.label.setAlignment(Qt.AlignmentFlag.AlignRight)
        layout.addWidget(self.text_label)
        layout.addWidget(self.label)
        self.setLayout(layout)
        self.starling_started = False

    def check_reachability(self):
        while not self.connection_widget.get_reached():
            pass
        starling_commands = {
            "source ~/aeroprint/install/setup.bash && ros2 launch starling starling_launch.py"
        }
        try: 
            ssh_thread = threading.Thread(target=self.run_ssh_commands_thread)
            ssh_thread.start()
            while not self.starling_started:
                pass
            self.label.setPixmap(QPixmap(self.check_path).scaled(self.icon_size, self.icon_size))
            self.checker.pass_check("starling_started")
        except Exception as e: 
            print(e)
            QTimer.singleShot(1000, self.check_reachability)

    def run_ssh_commands_thread(self):
        starling_command = {
            "source ~/aeroprint/install/setup.bash && ros2 launch starling starling_launch.py"
        }
        self.run_ssh_commands("m0054", "root", "oelinux123", starling_command)

    def run_ssh_commands(self, host, username, password, commands):
        host = "m0054"
        username = "root"
        password = "oelinux123"
        launch_command = "source ~/aeroprint/install/setup.bash && ros2 launch starling starling_launch.py"
        get_nodes_command = "source /opt/ros/foxy/setup.bash && ros2 node list"
        # Create SSH client
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        # Connect to the host
        client.connect(host, username=username, password=password)
        node_stdin, node_stdout, node_stderr = client.exec_command(get_nodes_command)
        if ("starling_fc" in node_stdout.read().decode()):
            message = """
            Starling Flight Controller already detected.\n
            If Starling does not fly as expected:\n
            - Reboot Starling
            - Restart this program
            """
            warning = Warning(message)
            warning.show()
            pass
        else: 
            launch_stdin, launch_stdout, launch_stderr = client.exec_command(launch_command)
        print("Ran launch command")
        
        count = 0
        count_max = 30
        while count < count_max:
            node_stdin, node_stdout, node_stderr = client.exec_command(get_nodes_command)
            if ("starling_fc" in node_stdout.read().decode()):
                print("YAY")
                self.starling_started = True
                count = count_max
            print(node_stdout.read().decode())

        client.close()

class Warning(QDialog):
    def __init__(self, message):
        super().__init__()
        self.setWindowTitle("Warning")
        self.text_label = QLabel(self)
        self.text_label.setText(message)
        layout = QVBoxLayout()
        layout.addWidget(self.text_label)
        self.setLayout(layout)

    def set_text(self, text):
        self.text_label.setText(text)


class WifiConnectionWidget(QWidget):
    def __init__(self, checker):
        super().__init__()
        self.checker = checker
        self.icon_size = 50
        layout = QHBoxLayout()
        
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
        self.text_label = QLabel(self)
        self.text_label.setText("AeroPrint Network Connected")
        self.text_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.check_path = os.path.join(truncated_path, 'src/host/images/check.png')  
        self.x_path = os.path.join(truncated_path, 'src/host/images/x.png')  
        self.label = QLabel(self)
        self.label.setPixmap(QPixmap(self.x_path).scaled(self.icon_size, self.icon_size))
        self.label.setAlignment(Qt.AlignmentFlag.AlignRight)
        layout.addWidget(self.text_label)
        layout.addWidget(self.label)
        self.setLayout(layout)

    def check_reachability(self):
        net_requirement = "aeroprint-net"
        wifi_ssid = get_wifi_ssid()
        try: 
            if net_requirement in wifi_ssid:
                self.label.setPixmap(QPixmap(self.check_path).scaled(self.icon_size, self.icon_size))
                self.checker.pass_check("wifi")
            else:
                self.label.setPixmap(QPixmap(self.x_path).scaled(self.icon_size, self.icon_size))
                # Wait for 1 second and check again
                QTimer.singleShot(1000, self.check_reachability)
        except: 
            QTimer.singleShot(1000, self.check_reachability)
            

class StarlingConnectionWidget(QWidget):
    def __init__(self, checker):
        super().__init__()
        self.checker = checker
        self.icon_size = 50
        layout = QHBoxLayout()
        
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
        self.text_label = QLabel(self)
        self.text_label.setText("Starling Connected")
        self.text_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.check_path = os.path.join(truncated_path, 'src/host/images/check.png')  
        self.x_path = os.path.join(truncated_path, 'src/host/images/x.png')  
        self.label = QLabel(self)
        self.label.setPixmap(QPixmap(self.x_path).scaled(self.icon_size, self.icon_size))
        self.label.setAlignment(Qt.AlignmentFlag.AlignRight)
        layout.addWidget(self.text_label)
        layout.addWidget(self.label)
        self.setLayout(layout)
        self.reached = False
        self.check_reachability()
        
    def get_reached(self):
        return self.reached

    def check_reachability(self):
        host = "m0054"
        if is_reachable(host):
            self.label.setPixmap(QPixmap(self.check_path).scaled(self.icon_size, self.icon_size))
            self.checker.pass_check("starling_connection")
            self.reached = True
        else:
            self.label.setPixmap(QPixmap(self.x_path).scaled(self.icon_size, self.icon_size))
            # Wait for 1 second and check again
            QTimer.singleShot(1000, self.check_reachability)
        
class Checker():
    def __init__(self):
        self.checks = {}

    def add_check(self, key):
        self.checks[key] = False

    def pass_check(self, key):
        self.checks[key] = True

    def check_passed(self, key):
        return self.checks[key]
    
    def all_checks_passed(self):
        for key in self.checks:
            if not self.checks[key]:
                return False
        return True

def is_reachable(host):
    try:
        subprocess.check_output(['ping', '-c', '1', host])
        return True
    except subprocess.CalledProcessError:
        return False
    
def get_wifi_ssid():
    try:
        output = subprocess.check_output(['iwgetid', '-r'])
        ssid = output.decode().strip()
        return ssid
    except subprocess.CalledProcessError:
        return None
     



app = QApplication([])
window = MainWindow()
window.show()

app.exec()