#! /usr/bin/env python3
# ROS MODULES
import rospy
import sys

from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget, QFileDialog, QGridLayout
from PyQt5.QtGui import QPixmap
from PyQt5 import QtGui, QtCore
from PyQt5.QtGui import QCursor



if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('crazy_app_node', log_level=rospy.DEBUG)

    app = QApplication(sys.argv)
    window = QWidget()
    window.setWindowTitle("CrazyApp")
    window.setFixedWidth(1000)
    window.setStyleSheet("background: #161219;")

    grid = QGridLayout()

    window.setLayout(grid)

    window.show()
    sys.exit(app.exec())
