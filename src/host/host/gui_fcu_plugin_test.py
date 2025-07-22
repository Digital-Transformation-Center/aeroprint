import sys
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLineEdit, QLabel
)
from PyQt6.QtCore import pyqtSlot, QTimer

import rclpy
from rclpy.node import Node

from gui_fcu_plugin import GUI_FCU_Plugin

class FCUGui(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.ros_node.set_heartbeat_alive_callback("heartbeat_alive", self)
        self.ros_node.set_heartbeat_dead_callback("heartbeat_dead", self)
        self._ros_spin_running = True
        import threading
        self._ros_spin_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
        self._ros_spin_thread.start()
        self.init_ui()

    def _ros_spin_loop(self):
        import time
        while self._ros_spin_running:
            rclpy.spin_once(self.ros_node, timeout_sec=0.01)
            time.sleep(0.05)  # 20Hz

    def closeEvent(self, event):
        self._ros_spin_running = False
        if self._ros_spin_thread.is_alive():
            self._ros_spin_thread.join(timeout=1)
        super().closeEvent(event)

    def init_ui(self):
        self.setWindowTitle('FCU Plugin Test')

        layout = QVBoxLayout()

        # Float value inputs
        self.radius_edit = QLineEdit()
        self.height_edit = QLineEdit()
        self.turns_edit = QLineEdit()
        self.start_height_edit = QLineEdit()
        layout.addWidget(QLabel('Radius:'))
        layout.addWidget(self.radius_edit)
        layout.addWidget(QLabel('Height:'))
        layout.addWidget(self.height_edit)
        layout.addWidget(QLabel('Turns:'))
        layout.addWidget(self.turns_edit)
        layout.addWidget(QLabel('Start Height:'))
        layout.addWidget(self.start_height_edit)


        # Buttons
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton('Start Flight')
        self.land_btn = QPushButton('Land')
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.land_btn)
        layout.addLayout(btn_layout)

        self.setLayout(layout)

        self.heartbeat_indicator = QLabel('Heartbeat: Dead')
        layout.addWidget(self.heartbeat_indicator)

        # Connect signals
        self.start_btn.clicked.connect(self.on_start_clicked)
        self.land_btn.clicked.connect(self.on_land_clicked)

        self.ros_node.run()


    def heartbeat_alive(self):
        self.heartbeat_indicator.setText('Heartbeat: Alive')

    def heartbeat_dead(self):
        self.heartbeat_indicator.setText('Heartbeat: Dead')

    

    @pyqtSlot()
    def on_start_clicked(self):
        try:
            radius = float(self.radius_edit.text())
            height = float(self.height_edit.text())
            turns = float(self.turns_edit.text())
            start_height = float(self.start_height_edit.text())
            self.ros_node.set_helix_params(radius, height, turns, start_height)

            starting = False
            while not starting:
                if self.ros_node.flight_path_loaded():
                    starting = True
                else:
                    self.ros_node.get_logger().info("Waiting for flight path to load...")
                    rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            self.ros_node.start_flight()
        except ValueError:
            self.ros_node.get_logger().warn('Invalid float input')

    @pyqtSlot()
    def on_land_clicked(self):
        self.radius_edit.clear()
        self.height_edit.clear()
        self.turns_edit.clear()
        self.start_height_edit.clear()
        self.ros_node.land()

# Heartbeat management
def set_heartbeat_alive_callback(self, callback):
    """Callback params: no arguments"""
    self._user_heartbeat_alive_callback = callback

def set_heartbeat_dead_callback(self, callback):
    """Callback params: no arguments"""
    self._user_heartbeat_dead_callback = callback

def main(args=None):
    rclpy.init(args=args)
    ros_node = GUI_FCU_Plugin()

    app = QApplication(sys.argv)
    gui = FCUGui(ros_node)
    gui.show()

    # Run Qt event loop
    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()