import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap
from PyQt5 import QtGui, QtCore
from PyQt5.QtGui import QCursor

from app_styles import *
from custom_widgets import MainMenuButton, MenuButton

def button_pressed():
    print('Button pressed!!')

def show2():
    launchfile_window.show()
    menu_window.hide()

app = QApplication(sys.argv)


# ======================================================================================================================
#
#                                               M E N U  W I N D O W
#
# ======================================================================================================================
# ----------------------------------------------------------------------------------------------------------------------
#                                             W I N D O W  S E T T I N G S
# ----------------------------------------------------------------------------------------------------------------------
menu_window = QWidget()
menu_window.setWindowTitle('Crazy App')
menu_window.setFixedWidth(500)

menu_window.setStyleSheet(f'background: {LIGHT_BACKGROUND_COLOR};')

menu_grid = QGridLayout()
# ----------------------------------------------------------------------------------------------------------------------
#                                                     L O G O
# ----------------------------------------------------------------------------------------------------------------------
logo_image = QPixmap('AppImages/Logo.png')
menu_logo = QLabel()
menu_logo.setPixmap(logo_image)
menu_logo.setAlignment(QtCore.Qt.AlignCenter)
menu_logo.setStyleSheet('margin-top: 10px;')

# ----------------------------------------------------------------------------------------------------------------------
#                                                    B U T T O N S
# ----------------------------------------------------------------------------------------------------------------------
launch_file_gen_button = MainMenuButton('Launch file generator', '50px', button_pressed)
export_urdf_button = MainMenuButton('Export URDF', '5px', button_pressed)
task_launcher_button = MainMenuButton('Task launcher', '5px', button_pressed)
settings_button = MainMenuButton('Settings', '5px', show2)

# ----------------------------------------------------------------------------------------------------------------------
#                                               A D D I N G  W I D J E T S
# ----------------------------------------------------------------------------------------------------------------------
menu_grid.addWidget(menu_logo, 0, 0)
menu_grid.addWidget(launch_file_gen_button.Button, 1, 0)
menu_grid.addWidget(export_urdf_button.Button, 2, 0)
menu_grid.addWidget(task_launcher_button.Button, 3, 0)
menu_grid.addWidget(settings_button.Button, 4, 0)


menu_window.setLayout(menu_grid)

# ======================================================================================================================
#
#                                               L A U N C H  F I L E  W I N D O W
#
# ======================================================================================================================
launchfile_window = QWidget()
launchfile_window.setWindowTitle('Crazy App - Launch File Generator')
launchfile_window.setFixedWidth(1000)
launchfile_window.setStyleSheet(f'background: {LIGHT_BACKGROUND_COLOR};')

launchfile_outer_layout = QVBoxLayout()
launchfile_menu_layout = QHBoxLayout()
launchfile_menu_layout.setSpacing(0)
launchfile_main_layout = QHBoxLayout()

menu_button = QLabel()
menu_button_image_blu = QPixmap('AppImages/Menu_icon_blu.png').scaledToHeight(30)
menu_button_image_white = QPixmap('AppImages/Menu_icon_white.png').scaledToHeight(30)
menu_button.setPixmap(menu_button_image_blu)
menu_button.setAlignment(QtCore.Qt.AlignLeft)
menu_button.setStyleSheet(f'background: {MENU_COLOR}; padding: 10px;')
launchfile_menu_layout.addWidget(menu_button)

launchfile_logo = QLabel()
menu_logo_image = QPixmap('AppImages/Logo_white.png').scaledToHeight(30)
launchfile_logo.setPixmap(menu_logo_image)
launchfile_logo.setAlignment(QtCore.Qt.AlignRight)
launchfile_logo.setStyleSheet(f'background: {MENU_COLOR}; padding: 10px;')
launchfile_menu_layout.addWidget(launchfile_logo)


launchfile_outer_layout.addLayout(launchfile_menu_layout)
launchfile_outer_layout.addLayout(launchfile_main_layout)

launchfile_window.setLayout(launchfile_outer_layout)


launchfile_window.show()
sys.exit(app.exec())