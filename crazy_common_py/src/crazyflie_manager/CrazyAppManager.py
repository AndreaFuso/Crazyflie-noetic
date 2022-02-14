import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap, QMouseEvent, QCursor, QPainter, QPen
from PyQt5 import QtGui, QtCore
from PyQt5.QtCore import QSize
from qtwidgets import Toggle

from app_styles import *
from custom_widgets import MainMenuButton, MyMenuBar, GenericSection, MyButton, PreviewWidget
from app_managers import PreviewLaunchManager, CrazyflieType, SwarmType

def button_pressed():
    print('Button pressed!!')

def show_launch_file_generator():
    launchfile_window.show()
    menu_window.hide()

def menu_button_pressed():
    menu_window.show()


app = QApplication(sys.argv)

def string2type(text):
    if text == 'Pyramid':
        return SwarmType.PYRAMID
    else:
        return SwarmType.GRID

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
    if lf_cf_number_spinner.value() > 1:
        lf_swarm_type_label.show()
        lf_swarm_type_list.show()
        if lf_swarm_type_list.currentText() == 'Pyramid':
            lf_grid_swarm_parameters.hide()
            lf_pyramid_swarm_parameters.show()
        else:
            lf_grid_swarm_parameters.show()
            lf_pyramid_swarm_parameters.hide()
    else:
        lf_swarm_type_label.hide()
        lf_swarm_type_list.hide()
        lf_grid_swarm_parameters.hide()
        lf_grid_swarm_parameters.hide()
        lf_pyramid_swarm_parameters.hide()

def add_node():
    print('New node added')
    actual_preview_widgets.append(PreviewWidget(lf_nodes_input_label.text(), update_preview))
    update_preview()

def add_launch():
    print('Launch file added')
    actual_preview_widgets.append(PreviewWidget(lf_add_lfs_input_label.text(), update_preview))
    update_preview()

def generate_launchfile():
    global lf_preview_manager
    print('Launch File correctly generated')
    lf_preview_manager.clear()

    for ii in range(0, len(actual_preview_widgets)):
        lf_preview_manager.addElementByString(actual_preview_widgets[ii].text)

    param = [[lf_pyramid_swarm_levels_spinner.value(),
                                                           float(lf_pyramid_swarm_distance_input.text()),
                                                           float(lf_pyramid_swarm_vdistance_input.text()),
                                                           float(lf_pyramid_swarm_z0_input.text())],
                                                          [lf_grid_swarm_cfx_spinner.value(),
                                                           lf_grid_swarm_cfy_spinner.value(),
                                                           float(lf_grid_swarm_xoff_input.text()),
                                                           float(lf_grid_swarm_yoff_input.text()),
                                                           float(lf_grid_swarm_z0_input.text())]]
    if lf_sim_toggle.checkState() != 0:
        lf_preview_manager.setSwarmProperties(cfs_number=lf_cf_number_spinner.value(), cf_type=CrazyflieType.SIMULATED,
                                              swarm_type=string2type(lf_swarm_type_list.currentText()),
                                              parameters=param[:])
        print(f'Filename: {lf_name_input_label.text()}; Simulated, {lf_cf_number_spinner.value()} cfs, {string2type(lf_swarm_type_list.currentText())} swarm')
    else:
        lf_preview_manager.setSwarmProperties(cfs_number=lf_cf_number_spinner.value(), cf_type=CrazyflieType.REAL,
                                              swarm_type=string2type(lf_swarm_type_list.currentText()),
                                              parameters=param[:])
        print(f'Filename: {lf_name_input_label.text()}; Real, {lf_cf_number_spinner.value()} cfs, {string2type(lf_swarm_type_list.currentText())} swarm')


    lf_preview_manager.generateFile(lf_name_input_label.text())

def generate_and_launch():
    print('Launching...')

def update_preview():
    global actual_preview_widgets
    for i in reversed(range(lf_preview_vbox.count())):
        lf_preview_vbox.itemAt(i).widget().setParent(None)

    actual_preview_widgets_cp = actual_preview_widgets[:]

    for ii in range(0, len(actual_preview_widgets)):
        if not actual_preview_widgets[ii].active:
            actual_preview_widgets_cp.pop(ii)

    actual_preview_widgets = actual_preview_widgets_cp[:]

    for ii in range(0, len(actual_preview_widgets)):
        lf_preview_vbox.addWidget(actual_preview_widgets[ii].widget)

    lf_preview_vbox.setAlignment(QtCore.Qt.AlignTop)

launchfile_window = QWidget()
launchfile_window.setWindowTitle('Crazy App - Launch File Generator')
launchfile_window.setFixedWidth(1000)
launchfile_window.setFixedHeight(700)
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
lf_name_input_label.setText('auto_launch')

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

# SWARM TYPE
lf_swarm_type_label = QLabel('Swarm type:')
lf_swarm_type_label.setStyleSheet(
    "font-weight: bold;"
)
lf_swarm_type_list = QComboBox()
lf_swarm_type_list.addItems(['Pyramid', 'Grid'])
lf_swarm_type_list.setStyleSheet(
    "*{border: 2px solid #1D3557;" +
    "border-radius: 3px;" +
    "selection-background-color: rgb(168,168,168);}" +
    "QComboBox::drop-down{" +
    "width: 30px;" +
    "height: 18px;" +
    "}" +
    "QComboBox::down-arrow::button {" +
    "width: 20px;" +
    "margin: -2px;" +
    "padding: 1px;" +
    "image: url(AppImages/drop_down_icon.png)" +
    "}"
)
lf_swarm_type_list.setFixedWidth(150)
lf_swarm_type_list.activated.connect(cf_number_cb)

lf_swarm_type_h_spacer = QSpacerItem(1000, 20, QSizePolicy.Maximum, QSizePolicy.Expanding)


lf_swarm_type_param = QHBoxLayout()
lf_swarm_type_param.addWidget(lf_swarm_type_label)
lf_swarm_type_param.addWidget(lf_swarm_type_list)
lf_swarm_type_param.addItem(lf_swarm_type_h_spacer)


# GRID PARAMETERS
# CFx
lf_grid_swarm_parameters = QWidget()
lf_grid_swarm_cfx_label = QLabel('n° cfs X side:')
lf_grid_swarm_cfx_label.setStyleSheet(
    '''
    font-weight: bold;
    '''
)
lf_grid_swarm_cfx_spinner = QSpinBox()
lf_grid_swarm_cfx_spinner.setFixedWidth(70)

lf_grid_swarm_cfx_layout = QHBoxLayout()
lf_grid_swarm_cfx_layout.addWidget(lf_grid_swarm_cfx_label)
lf_grid_swarm_cfx_layout.addWidget(lf_grid_swarm_cfx_spinner)
lf_grid_swarm_cfx_widget = QWidget()
lf_grid_swarm_cfx_widget.setLayout(lf_grid_swarm_cfx_layout)

# cfY
lf_grid_swarm_cfy_label = QLabel('n° cfs Y side:')
lf_grid_swarm_cfy_label.setStyleSheet(
    '''
    font-weight: bold;
    '''
)
lf_grid_swarm_cfy_spinner = QSpinBox()
lf_grid_swarm_cfy_spinner.setFixedWidth(70)

lf_grid_swarm_cfy_layout = QHBoxLayout()
lf_grid_swarm_cfy_layout.addWidget(lf_grid_swarm_cfy_label)
lf_grid_swarm_cfy_layout.addWidget(lf_grid_swarm_cfy_spinner)
lf_grid_swarm_cfy_widget = QWidget()
lf_grid_swarm_cfy_widget.setLayout(lf_grid_swarm_cfy_layout)

# X offset
lf_grid_swarm_xoff_label = QLabel('X offset [m]:')
lf_grid_swarm_xoff_label.setStyleSheet(
    '''
    font-weight: bold;
    '''
)
lf_grid_swarm_xoff_input = QLineEdit()
lf_grid_swarm_xoff_input.setText('1.0')
lf_grid_swarm_xoff_input.setFixedWidth(70)

lf_grid_swarm_xoff_layout = QHBoxLayout()
lf_grid_swarm_xoff_layout.addWidget(lf_grid_swarm_xoff_label)
lf_grid_swarm_xoff_layout.addWidget(lf_grid_swarm_xoff_input)
lf_grid_swarm_xoff_widget = QWidget()
lf_grid_swarm_xoff_widget.setLayout(lf_grid_swarm_xoff_layout)

# Y offset
lf_grid_swarm_yoff_label = QLabel('Y offset [m]:')
lf_grid_swarm_yoff_label.setStyleSheet(
    '''
    font-weight: bold;
    '''
)
lf_grid_swarm_yoff_input = QLineEdit()
lf_grid_swarm_yoff_input.setText('1.0')
lf_grid_swarm_yoff_input.setFixedWidth(70)

lf_grid_swarm_yoff_layout = QHBoxLayout()
lf_grid_swarm_yoff_layout.addWidget(lf_grid_swarm_yoff_label)
lf_grid_swarm_yoff_layout.addWidget(lf_grid_swarm_yoff_input)
lf_grid_swarm_yoff_widget = QWidget()
lf_grid_swarm_yoff_widget.setLayout(lf_grid_swarm_yoff_layout)

# spawning altitude:
lf_grid_swarm_z0_label = QLabel('Spawning altitude [m]:')
lf_grid_swarm_z0_label.setStyleSheet(
    '''
    font-weight: bold;
    '''
)
lf_grid_swarm_z0_input = QLineEdit()
lf_grid_swarm_z0_input.setText('0.2')
lf_grid_swarm_z0_input.setFixedWidth(70)

lf_grid_swarm_z0_layout = QHBoxLayout()
lf_grid_swarm_z0_layout.addWidget(lf_grid_swarm_z0_label)
lf_grid_swarm_z0_layout.addWidget(lf_grid_swarm_z0_input)
lf_grid_swarm_z0_widget = QWidget()
lf_grid_swarm_z0_widget.setLayout(lf_grid_swarm_z0_layout)

lf_grid_swarm_parameters_layout = QGridLayout()
lf_grid_swarm_parameters_layout.addWidget(lf_grid_swarm_cfx_widget, 0, 0)
lf_grid_swarm_parameters_layout.addWidget(lf_grid_swarm_cfy_widget, 0, 1)
lf_grid_swarm_parameters_layout.addWidget(lf_grid_swarm_xoff_widget, 1, 0)
lf_grid_swarm_parameters_layout.addWidget(lf_grid_swarm_yoff_widget, 1, 1)
lf_grid_swarm_parameters_layout.addWidget(lf_grid_swarm_z0_widget, 2, 0)

lf_grid_swarm_parameters.setLayout(lf_grid_swarm_parameters_layout)

# PYRAMID SWARM
# levels
lf_pyramid_swarm_parameters = QWidget()
lf_pyramid_swarm_levels_label = QLabel('n° of levels:')
lf_pyramid_swarm_levels_label.setStyleSheet(
    '''
    font-weight: bold;
    '''
)
lf_pyramid_swarm_levels_spinner = QSpinBox()
lf_pyramid_swarm_levels_spinner.setFixedWidth(70)

lf_pyramid_swarm_levels_layout = QHBoxLayout()
lf_pyramid_swarm_levels_layout.addWidget(lf_pyramid_swarm_levels_label)
lf_pyramid_swarm_levels_layout.addWidget(lf_pyramid_swarm_levels_spinner)
lf_pyramid_swarm_levels_widget = QWidget()
lf_pyramid_swarm_levels_widget.setLayout(lf_pyramid_swarm_levels_layout)

# distance
lf_pyramid_swarm_distance_label = QLabel('cfs offset [m]:')
lf_pyramid_swarm_distance_label.setStyleSheet(
    '''
    font-weight: bold;
    '''
)
lf_pyramid_swarm_distance_input = QLineEdit()
lf_pyramid_swarm_distance_input.setText('1.0')
lf_pyramid_swarm_distance_input.setFixedWidth(70)

lf_pyramid_swarm_distance_layout = QHBoxLayout()
lf_pyramid_swarm_distance_layout.addWidget(lf_pyramid_swarm_distance_label)
lf_pyramid_swarm_distance_layout.addWidget(lf_pyramid_swarm_distance_input)
lf_pyramid_swarm_distance_widget = QWidget()
lf_pyramid_swarm_distance_widget.setLayout(lf_pyramid_swarm_distance_layout)

# vertical distance
lf_pyramid_swarm_vdistance_label = QLabel('vertical offset [m]:')
lf_pyramid_swarm_vdistance_label.setStyleSheet(
    '''
    font-weight: bold;
    '''
)
lf_pyramid_swarm_vdistance_input = QLineEdit()
lf_pyramid_swarm_vdistance_input.setText('1.0')
lf_pyramid_swarm_vdistance_input.setFixedWidth(70)

lf_pyramid_swarm_vdistance_layout = QHBoxLayout()
lf_pyramid_swarm_vdistance_layout.addWidget(lf_pyramid_swarm_vdistance_label)
lf_pyramid_swarm_vdistance_layout.addWidget(lf_pyramid_swarm_vdistance_input)
lf_pyramid_swarm_vdistance_widget = QWidget()
lf_pyramid_swarm_vdistance_widget.setLayout(lf_pyramid_swarm_vdistance_layout)

# spawning altitude
lf_pyramid_swarm_z0_label = QLabel('Spawning altitude [m]:')
lf_pyramid_swarm_z0_label.setStyleSheet(
    '''
    font-weight: bold;
    '''
)
lf_pyramid_swarm_z0_input = QLineEdit()
lf_pyramid_swarm_z0_input.setText('0.2')
lf_pyramid_swarm_z0_input.setFixedWidth(70)

lf_pyramid_swarm_z0_layout = QHBoxLayout()
lf_pyramid_swarm_z0_layout.addWidget(lf_pyramid_swarm_z0_label)
lf_pyramid_swarm_z0_layout.addWidget(lf_pyramid_swarm_z0_input)
lf_pyramid_swarm_z0_widget = QWidget()
lf_pyramid_swarm_z0_widget.setLayout(lf_pyramid_swarm_z0_layout)

lf_pyramid_swarm_parameters_layout = QGridLayout()
lf_pyramid_swarm_parameters_layout.addWidget(lf_pyramid_swarm_levels_widget, 0, 0)
lf_pyramid_swarm_parameters_layout.addWidget(lf_pyramid_swarm_distance_widget, 0, 1)
lf_pyramid_swarm_parameters_layout.addWidget(lf_pyramid_swarm_vdistance_widget, 1, 0)
lf_pyramid_swarm_parameters_layout.addWidget(lf_pyramid_swarm_z0_widget, 1, 1)

lf_pyramid_swarm_parameters.setLayout(lf_pyramid_swarm_parameters_layout)

if lf_cf_number_spinner.value() > 1:
    lf_swarm_type_label.show()
    lf_swarm_type_list.show()
    lf_grid_swarm_parameters.show()
else:
    lf_swarm_type_label.hide()
    lf_swarm_type_list.hide()
    lf_grid_swarm_parameters.hide()
    lf_pyramid_swarm_parameters.hide()

# VERTICAL SPACER:
lf_parameters_v_spacer = QSpacerItem(1000, 1000, QSizePolicy.Maximum, QSizePolicy.Expanding)

# Adding parameters section:
lf_parameters.addLayout(lf_name_param)
lf_parameters.addLayout(lf_real_sim_param)
lf_parameters.addLayout(lf_cf_number_param)
lf_parameters.addLayout(lf_nodes_param)
lf_parameters.addLayout(lf_add_lfs_param)
lf_parameters.addLayout(lf_swarm_type_param)

lf_parameters.addWidget(lf_grid_swarm_parameters)
lf_parameters.addWidget(lf_pyramid_swarm_parameters)

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

actual_preview_widgets = []

'''for ii in range(1, 50):
    object = PreviewWidget(f'Text {ii}', button_pressed).widget
    lf_preview_vbox.addWidget(object)'''

actual_preview_widgets.append(PreviewWidget('crazyCmd:node_100Hz:node_100Hz', update_preview))
actual_preview_widgets.append(PreviewWidget('crazyCmd:node_500Hz:node_500Hz', update_preview))
actual_preview_widgets.append(PreviewWidget('crazyCmd:node_1000Hz:node_1000Hz', update_preview))
update_preview()

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
lf_launch_btns_layout = QHBoxLayout()

# Generate launchfile button:
lf_generate_lf_btn = MyButton('Generate', '2px', generate_launchfile).Button

# Generate and launch button:
lf_generate_and_launch_lf_btn = MyButton('Generate and launch', '2px', generate_and_launch).Button

lf_launch_btns_layout.addWidget(lf_generate_lf_btn)
lf_launch_btns_layout.addWidget(lf_generate_and_launch_lf_btn)

lf_master_layout.addLayout(lf_launch_btns_layout)

# SPACER
vertical_spacer = QSpacerItem(1000, 20, QSizePolicy.Maximum, QSizePolicy.Expanding)

lf_master_layout.addItem(vertical_spacer)
launchfile_window.setLayout(lf_master_layout)

launchfile_window.show()
sys.exit(app.exec())