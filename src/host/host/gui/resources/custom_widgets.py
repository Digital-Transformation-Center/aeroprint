from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton, QMessageBox
from PyQt5.QtCore import Qt

class CenteredButton(QWidget):
    def __init__(self, text, parent=None):
        super().__init__(parent)

        layout = QHBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        self.setLayout(layout)

        self.button = QPushButton(text)
        self.clicked = self.button.clicked
        layout.addWidget(self.button)
