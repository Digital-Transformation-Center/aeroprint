import sys
from host.gui.scroll_container import ScrollContainer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLineEdit, QPushButton, QApplication, QAction, QToolBar, QDialog, QHBoxLayout
from host.gui.gui_pages.settings.settings_widget import SettingsWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIcon
from host.gui.resources.custom_widgets import CenteredButton
from host.gui.gui_pages.tensor_flow.ml_ui import MLUI
from host.gui.gui_pages.flight.flight_widget import FlightWidget
from host.gui.resources.settings_utility import SettingsUtility

import os
import rclpy
# from gui_pages.tensor_flow.camera_dump import CameraDumpGUI


class AeroPrintGUI:
    """
    AeroPrintGUI is a class that creates a GUI application using PyQt5. It initializes the application, sets up a menu bar, 
    and provides functionality to open a settings dialog. The class also creates custom widgets and adds them to a ScrollContainer.
    
    Attributes:
        app (QApplication): The main application instance.
        sc (ScrollContainer): The main scroll container for the GUI.
        settings_utility (SettingsUtility): Utility for accessing settings.
        resource_path (str): Path to the resources file.
    """
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.sc = ScrollContainer()
        self.init_menu_bar()
        self.settings_utility = SettingsUtility()
        self.resource_path = os.path.abspath(self.settings_utility.get_value("resources_file_path"))

    def init_menu_bar(self):
        """
        Initializes the menu bar for the AeroPrint GUI.
        This method creates a menu bar and adds an "AeroPrint" menu to it. 
        It also adds a "Settings" action to the "AeroPrint" menu, which 
        triggers the open_settings method when selected.
        """

        menu_bar = self.sc.menuBar()

        aeroprint_menu = menu_bar.addMenu("AeroPrint")
        self.sc.setMenuBar(menu_bar)

        settings_action = QAction("Settings", self.sc)
        settings_action.triggered.connect(self.open_settings)
        aeroprint_menu.addAction(settings_action)

    def open_settings(self):
        """
        Opens the settings dialog.
        This method creates and displays a modal dialog window with the title "Settings".
        The dialog contains a SettingsWidget and has a fixed size of 800x900 pixels.
        """

        dialog = QDialog(self.sc)
        dialog.setWindowTitle("Settings")
        dialog.setFixedSize(800, 900)

        # Add your widget to the dialog
        widget = SettingsWidget()
        layout = QVBoxLayout(dialog)
        layout.addWidget(widget)

        dialog.exec_()
    def run(self):
        """
        Runs the main application logic.
        This method creates custom widgets, adds them to a ScrollContainer, and sets up navigation buttons.
        It initializes and displays the GUI, then starts the application's event loop.
        
        Custom Widgets:

            - MyWidget: A custom widget with a text box and a centered button.
            - MLUI: A widget initialized with specific parameters.
            - FlightWidget: A widget initialized with resource path and other parameters.
        
        Navigation:

            - Adds a "Next" button to the first custom widget.
            - Adds a "Previous" button to the second custom widget.
        
        ScrollContainer:

            - Adds the custom widgets to the ScrollContainer.
            - Creates page buttons for navigation.
            - Displays the ScrollContainer.
        
        Exits:

            - Terminates the application when the event loop ends.
        """

        # Create a custom widget that will be added to the ScrollContainer
        class MyWidget(QWidget):
            def __init__(self):
                super().__init__()
                self.setFixedSize(600, 400)
                self.init_ui()
                # self.setStyleSheet(style)

            def init_ui(self):
                layout = QVBoxLayout()
                layout.setAlignment(Qt.AlignCenter)
                self.setLayout(layout)

                text_box = QLineEdit()
                layout.addWidget(text_box)

                # button_container = QWidget()
                # button_layout = QHBoxLayout()
                # button_layout.setAlignment(Qt.AlignCenter)
                # button = QPushButton("Button")
                # button_layout.addWidget(button)
                # button_container.setLayout(button_layout)
                button = CenteredButton("Button")
                layout.addWidget(button)


        # Create an instance of MyWidget and add it to the ScrollContainer
        widget0 = MLUI(self.sc, "fruits", "banana")
        widget00 = FlightWidget(self.resource_path, self.sc, widget0)
        
        widget = MyWidget()
        # Add a next button to the widget
        widget.layout().addWidget(self.sc.next_button())
        widget2 = MyWidget()
        # widget3 = CameraDumpGUI()
        # Add a previous button to the widget
        widget2.layout().addWidget(self.sc.previous_button())
        widgets = [widget00, widget0, widget, widget2]
        # Add the widgets to the ScrollContainer
        for widget in widgets:
            self.sc.add_page(widget)
        # Create the page buttons and show the ScrollContainer
        self.sc.create_page_buttons()
        self.sc.show()
        sys.exit(self.app.exec_())

def main(args=None):
    rclpy.init(args=args)
    AeroPrintGUI().run()

if __name__ == "__main__":
    main()
