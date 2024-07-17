import sys

from scroll_container import ScrollContainer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLineEdit, QPushButton, QApplication


"""
This script demonstrates how to use the ScrollContainer class to create a multi-page GUI.
"""

class MainGUIExample:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.sc = ScrollContainer()

    def run(self):
        # Create a custom widget that will be added to the ScrollContainer
        class MyWidget(QWidget):
            def __init__(self, style):
                super().__init__()
                self.setFixedSize(600, 400)
                self.init_ui()
                self.setStyleSheet(style)

            def init_ui(self):
                layout = QVBoxLayout()
                self.setLayout(layout)

                text_box = QLineEdit()
                layout.addWidget(text_box)

                button = QPushButton("Button")
                layout.addWidget(button)

        # Create an instance of MyWidget and add it to the ScrollContainer
        widget = MyWidget(self.sc.get_stylesheet())
        # Add a next button to the widget
        widget.layout().addWidget(self.sc.next_button())
        widget2 = MyWidget(self.sc.get_stylesheet())
        # Add a previous button to the widget
        widget2.layout().addWidget(self.sc.previous_button())
        widgets = [widget, widget2]
        # Add the widgets to the ScrollContainer
        for widget in widgets:
            self.sc.add_page(widget)
        # Create the page buttons and show the ScrollContainer
        self.sc.create_page_buttons()
        self.sc.show()
        sys.exit(self.app.exec_())

if __name__ == "__main__":
    MainGUIExample().run()
