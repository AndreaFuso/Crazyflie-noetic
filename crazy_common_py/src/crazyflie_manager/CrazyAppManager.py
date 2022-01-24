import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap, QMouseEvent, QCursor, QPainter, QPen
from PyQt5 import QtGui, QtCore
from PyQt5.QtCore import QSize
from qtwidgets import Toggle

from app_styles import *
from custom_widgets import MainMenuButton, MyMenuBar, GenericSection, MyButton, PreviewWidget
from app_managers import PreviewLaunchManager

def button_pressed():
    print('Button pressed!!')

def show_launch_file_generator():
    launchfile_window.show()
    menu_window.hide()

def menu_button_pressed():
    menu_window.show()


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
launch_file_gen_button = MyButton('Launch file generator', '50px', show_launch_file_generator)
export_urdf_button = MyButton('Export URDF', '5px', button_pressed)
task_launcher_button = MyButton('Task launcher', '5px', button_pressed)
settings_button = MyButton('Settings', '5px', button_pressed)

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
# ----------------------------------------------------------------------------------------------------------------------
#                                               F U N C T I O N S  & S E T T I N G S
# ----------------------------------------------------------------------------------------------------------------------
def sim_toggle_cb():
    if lf_real_toggle.checkState() == 0:
        lf_real_toggle.setChecked(True)
    else:
        lf_real_toggle.setChecked(False)
def real_toggle_cb():
    if lf_sim_toggle.checkState() == 0:
        lf_sim_toggle.setChecked(True)
    else:
        lf_sim_toggle.setChecked(False)

def cf_number_cb():
    print('Modified')

def add_node():
    print('New node added')

def add_launch():
    print('Launch file added')

def generate_launchfile():
    print('Launch File correctly generated')

def generate_and_launch():
    print('Launching...')

launchfile_window = QWidget()
launchfile_window.setWindowTitle('Crazy App - Launch File Generator')
launchfile_window.setFixedWidth(1000)
launchfile_window.setFixedHeight(500)
launchfile_window.setStyleSheet(f'background: {LIGHT_BACKGROUND_COLOR};')

# Master Layout:
lf_master_layout = QVBoxLayout()

# Titles layouts
lf_titles = QHBoxLayout()
# Main layout (parameters + preview):
launchfile_main_layout = QHBoxLayout()

# ----------------------------------------------------------------------------------------------------------------------
#                                               M E N U B A R
# ----------------------------------------------------------------------------------------------------------------------
launchfile_menubar = MyMenuBar(default_image_path='AppImages/Menu_icon_blu.png',
                               hover_image_path='AppImages/Menu_icon_white.png',
                               callback_function=menu_button_pressed)
lf_master_layout.addLayout(launchfile_menubar.menubar)
# ----------------------------------------------------------------------------------------------------------------------
#                                               T I T L E S
# ----------------------------------------------------------------------------------------------------------------------
# PARAMETERS
lf_parameters_title = GenericSection(200, 0, 'PARAMETERS').section_layout
lf_titles.addLayout(lf_parameters_title)
lf_titles.setAlignment(QtCore.Qt.AlignTop)

# VERTICAL LINE
lf_titles_vertical_line = QLabel()
lf_titles_vertical_line.setFixedWidth(5)
lf_titles_vertical_line.setStyleSheet(
    f"background: {LIGHT_BACKGROUND_COLOR};"
    "border-radius: 2px;"
)
lf_titles.addWidget(lf_titles_vertical_line)

# PREVIEW:
lf_preview_title = GenericSection(225, 0, 'PREVIEW').section_layout
lf_titles.addLayout(lf_preview_title)


lf_master_layout.addLayout(lf_titles)
# ----------------------------------------------------------------------------------------------------------------------
#                                               P A R A M E T E R S  S E C T I O N
# ----------------------------------------------------------------------------------------------------------------------
lf_parameters = QVBoxLayout()

# NAME PARAMETER
lf_name_label = QLabel('Name:')
lf_name_label.setStyleSheet(
    "font-weight: bold;"
)
lf_name_input_label = QLineEdit()
lf_name_input_label.setText('my_launch')

lf_name_param = QHBoxLayout()
lf_name_param.addWidget(lf_name_label)
lf_name_param.addWidget(lf_name_input_label)

# REAL OR SIMULATED
lf_sim_param_label = QLabel('Simulated')
lf_sim_param_label.setStyleSheet(
    "font-weight: bold;"
)
lf_sim_toggle = Toggle()
lf_sim_toggle.setFixedWidth(75)
lf_sim_toggle.setChecked(True)
lf_sim_toggle.clicked.connect(sim_toggle_cb)

lf_real_param_label = QLabel('Real')
lf_real_param_label.setStyleSheet(
    "font-weight: bold;"
)
lf_real_toggle = Toggle()
lf_real_toggle.setFixedWidth(75)
lf_real_toggle.setChecked(False)
lf_real_toggle.clicked.connect(real_toggle_cb)

lf_real_sim_h_spacer = QSpacerItem(1000, 20, QSizePolicy.Maximum, QSizePolicy.Expanding)

lf_real_sim_param = QHBoxLayout()
lf_real_sim_param.addWidget(lf_sim_param_label)
lf_real_sim_param.addWidget(lf_sim_toggle)
lf_real_sim_param.addWidget(lf_real_param_label)
lf_real_sim_param.addWidget(lf_real_toggle)
lf_real_sim_param.addItem(lf_real_sim_h_spacer)

# NUMBER OF CRAZYFLIES:
lf_cf_number_label = QLabel('Number of crazyflies:')
lf_cf_number_label.setStyleSheet(
    "font-weight: bold;"
)
lf_cf_number_spinner = QSpinBox()
lf_cf_number_spinner.setValue(1)
lf_cf_number_spinner.valueChanged.connect(cf_number_cb)
lf_cf_number_spinner.setFixedWidth(75)

lf_cf_number_h_spacer = QSpacerItem(1000, 20, QSizePolicy.Maximum, QSizePolicy.Expanding)

lf_cf_number_param = QHBoxLayout()
lf_cf_number_param.addWidget(lf_cf_number_label)
lf_cf_number_param.addWidget(lf_cf_number_spinner)
lf_cf_number_param.addItem(lf_cf_number_h_spacer)

# ADDITIONAL NODES:
lf_nodes_label = QLabel('Add node:')
lf_nodes_label.setStyleSheet(
    "font-weight: bold;"
)
lf_nodes_input_label = QLineEdit()
lf_nodes_input_label.setText('PackageName:NodeName:ScriptName')
lf_nodes_input_label.setFixedWidth(300)
lf_nodes_add_btn = MyButton('Add', '5px', add_node).Button
lf_nodes_add_btn.setFixedWidth(50)

lf_nodes_h_spacer = QSpacerItem(1000, 20, QSizePolicy.Maximum, QSizePolicy.Expanding)

lf_nodes_param = QHBoxLayout()
lf_nodes_param.addWidget(lf_nodes_label)
lf_nodes_param.addWidget(lf_nodes_input_label)
lf_nodes_param.addWidget(lf_nodes_add_btn)
#lf_nodes_param.addItem(lf_nodes_h_spacer)

# ADDITIONAL LAUNCH FILES
lf_add_lfs_label = QLabel('Add launch file:')
lf_add_lfs_label.setStyleSheet(
    "font-weight: bold;"
)
lf_add_lfs_input_label = QLineEdit()
lf_add_lfs_input_label.setText('PackageName:LaunchFileName')
lf_add_lfs_input_label.setFixedWidth(300)
lf_add_lfs_add_btn = MyButton('Add', '5px', add_launch).Button
lf_add_lfs_add_btn.setFixedWidth(50)

lf_add_lfs_h_spacer = QSpacerItem(1000, 20, QSizePolicy.Maximum, QSizePolicy.Expanding)

lf_add_lfs_param = QHBoxLayout()
lf_add_lfs_param.addWidget(lf_add_lfs_label)
lf_add_lfs_param.addWidget(lf_add_lfs_input_label)
lf_add_lfs_param.addWidget(lf_add_lfs_add_btn)
#lf_add_lfs_param.addItem(lf_add_lfs_h_spacer)

# VERTICAL SPACER:
lf_parameters_v_spacer = QSpacerItem(1000, 1000, QSizePolicy.Maximum, QSizePolicy.Expanding)

# Adding parameters section:
lf_parameters.addLayout(lf_name_param)
lf_parameters.addLayout(lf_real_sim_param)
lf_parameters.addLayout(lf_cf_number_param)
lf_parameters.addLayout(lf_nodes_param)
lf_parameters.addLayout(lf_add_lfs_param)

lf_parameters.addItem(lf_parameters_v_spacer)
launchfile_main_layout.addLayout(lf_parameters)

# VERTICAL LINE
lf_parameters_vertical_line = QLabel()
lf_parameters_vertical_line.setFixedWidth(5)
lf_parameters_vertical_line.setStyleSheet(
    f"background: #D9D9D9;"
    "border-radius: 2px;"
)
launchfile_main_layout.addWidget(lf_parameters_vertical_line)
# ----------------------------------------------------------------------------------------------------------------------
#                                               P R E V I E W  S E C T I O N
# ----------------------------------------------------------------------------------------------------------------------
lf_preview_manager = PreviewLaunchManager()

lf_preview_scroll = QScrollArea()
lf_preview_scroll.setFixedWidth(480)
lf_preview_widget = QWidget()
lf_preview_vbox = QVBoxLayout()

'''for ii in range(1, 50):
    object = PreviewWidget(f'Text {ii}').widget
    lf_preview_vbox.addWidget(object)'''

lf_preview_widget.setLayout(lf_preview_vbox)

lf_preview_scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
lf_preview_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
lf_preview_scroll.setWidgetResizable(True)
lf_preview_scroll.setWidget(lf_preview_widget)

lf_preview_scroll.setStyleSheet('''
    QScrollBar:vertical {
        background: #CED4DA;
        border-radius: 5px;
    }
    QScrollBar::handle:vertical {
        margin: 2px;
        background: #457B9D;
        border-radius: 5px;
        
    }
    QScrollBar::add-line:vertical {
        height: 0px;
    }
    
    QScrollBar::sub-line:vertical {
        height: 0px;
    }
    
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
        height: 0px;
    }
'''

)

launchfile_main_layout.addWidget(lf_preview_scroll)


lf_master_layout.addLayout(launchfile_main_layout)
# ----------------------------------------------------------------------------------------------------------------------
#                                               B U T T O N S  S E C T I O N
# ----------------------------------------------------------------------------------------------------------------------

# SPACER
vertical_spacer = QSpacerItem(1000, 20, QSizePolicy.Maximum, QSizePolicy.Expanding)

lf_master_layout.addItem(vertical_spacer)
launchfile_window.setLayout(lf_master_layout)

launchfile_window.show()
sys.exit(app.exec())