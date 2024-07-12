import sys
from PyQt5.QtGui import QColor
from PyQt5.QtCore import pyqtProperty
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel, QStackedWidget, QHBoxLayout
import os
from PyQt5.QtCore import pyqtProperty, QPropertyAnimation
from PyQt5.QtCore import pyqtProperty, QPoint
from PyQt5.QtCore import QEasingCurve
from resources.custom_widgets import CenteredButton

class ScrollContainer(QMainWindow):
    def __init__(self):
        super().__init__()
            
        self.setWindowTitle("AeroPrint")
        self.setGeometry(100, 100, 400, 300)
        self.index = 0
        
        # Load style sheet
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
        style_path = os.path.join(truncated_path, "src/host/host/gui/resources/style.qss")

        with open(style_path, "r") as fh:
            self.style_sheet = fh.read()
            self.setStyleSheet(self.style_sheet)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        self.stacked_widget = QStackedWidget()

        self.layout.addWidget(self.stacked_widget)
        self.page_buttons = []
        self.ps = PageSwitcher(self.stacked_widget, self.page_buttons)

        self.page_buttons_layout = QHBoxLayout()
        self.layout.addLayout(self.page_buttons_layout)


        
        

    def create_page_buttons(self):
        self.page_buttons_layout.addStretch(1)  # Add stretchable space before buttons
        
    
        for i in range(self.stacked_widget.count()):
            # button = QPushButton()
            button = PageButton(i)
            button.setObjectName("page-button")

            button.clicked.connect(lambda state, x=i: self.set_page(x))
            self.page_buttons.append(button)

            self.page_buttons_layout.addWidget(button)

        self.page_buttons_layout.addStretch(1)  # Add stretchable space after buttons
        self.ps.update_buttons()

    def next_button(self, text="Next"):
        button = CenteredButton(text)
        button.button.setObjectName("action-button")
        button.clicked.connect(self.ps.next)
        return button

    def previous_button(self, text="Previous"):
        button = CenteredButton(text)
        button.button.setObjectName("action-button")
        button.clicked.connect(self.ps.previous)
        return button

    def add_page(self, page):
        self.stacked_widget.addWidget(page)

    def set_page(self, index):
        self.ps.set_index(index)

    def next(self):
        self.ps.next()
    
    def previous(self):
        self.ps.previous()


    def get_stylesheet(self):
        return self.style_sheet

class TestPage(QWidget):
    def __init__(self, page_switcher):
        super().__init__()
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.label = QLabel("Test Page")
        self.layout.addWidget(self.label)
        self.next_button = QPushButton("Next")
        self.next_button.clicked.connect(page_switcher.next)
        self.layout.addWidget(self.next_button)


class PageSwitcher():
    def __init__(self, stacked_widget, buttons):
        self.index = 0
        self.stacked_widget = stacked_widget
        self.buttons = buttons

    def set_index(self, index):
        self.stacked_widget.setCurrentIndex(index)
        self.index = index
        self.update_buttons()

    def get_index(self):
        return self.index
    
    def update_buttons(self):
        for button in self.buttons:
            button.current = False
        self.buttons[self.index].current = True

    def set_stacked_widget(self, stacked_widget):
        self.stacked_widget = stacked_widget

    def next(self):
        self.index += 1
        if self.index >= self.stacked_widget.count():
            self.index=  0
        self.set_index(self.index)

    def previous(self):
        self.index -= 1
        if self.index < 0:
            self.index = self.stacked_widget.count() - 1
        self.set_index(self.index)


class PageButton(QPushButton):
    def __init__(self, index):
        super().__init__()
        self.index = index
        self.setAutoFillBackground(True)
        self._current = False

    def set_index(self, index):
        self.index = index

    def get_index(self):
        return self.index

    @pyqtProperty(bool)
    def current(self):
        return self._current
    
    @current.setter
    def current(self, value):
        self._current = value
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ScrollContainer()
    window.show()
    sys.exit(app.exec_())