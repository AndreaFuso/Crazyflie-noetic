import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap
from PyQt5 import QtGui, QtCore
from PyQt5.QtGui import QCursor

class MenuButton():
    def __init__(self, text, margin_top):
        self.Button = QPushButton(text)
        self.Button.setCursor(QCursor(QtCore.Qt.PointingHandCursor))
        self.Button.setStyleSheet(
            "*{border: 4px solid '#457B9D';" +
            "border-radius: 10px;" +
            "font-size: 15px;" +
            "font-family: Tahoma, sans-serif;" +
            "font-weight: bold;" +
            "color: 'black';" +
            "background-color: rgba(69,123,157,0.6);" +
            "padding: 5px 0;" +
            f"margin-top: {margin_top}px;" +
            "}" +
            "*:hover{background: '#457B9D'; color: '#F1FAEE';}"
        )


app = QApplication(sys.argv)

window = QWidget()
window.setWindowTitle('Crazy App')
window.setFixedWidth(500)

window.setStyleSheet('background: #F1FAEE;')

grid = QGridLayout()
# Display logo:
image = QPixmap('AppImages/Logo.png')
logo = QLabel()
logo.setPixmap(image)
logo.setAlignment(QtCore.Qt.AlignCenter)
logo.setStyleSheet('margin-top: 10px;')

# Button widget:
'''launch_file_gen_button = QPushButton('Launch file generator')
launch_file_gen_button.setCursor(QCursor(QtCore.Qt.PointingHandCursor))
launch_file_gen_button.setStyleSheet(
    "*{border: 4px solid '#457B9D';" +
    "border-radius: 10px;" +
    "font-size: 15px;" +
    "font-family: Tahoma, sans-serif;" +
    "font-weight: bold;" +
    "color: 'black';" +
    "background-color: rgba(69,123,157,0.6);" +
    "padding: 5px 0;" +
    "margin-top: 50px;}" +
    "*:hover{background: '#457B9D'; color: '#F1FAEE';}"
)'''
launch_file_gen_button = MenuButton('Launch file generator', 50)
export_urdf_button = MenuButton('Export URDF', 5)
task_launcher_button = MenuButton('Task launcher', 5)
settings_button = MenuButton('Settings', 5)

grid.addWidget(logo, 0, 0)
grid.addWidget(launch_file_gen_button.Button, 1, 0)
grid.addWidget(export_urdf_button.Button, 2, 0)
grid.addWidget(task_launcher_button.Button, 3, 0)
grid.addWidget(settings_button.Button, 4, 0)
window.setLayout(grid)

window.show()
sys.exit(app.exec())