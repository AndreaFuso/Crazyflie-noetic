from app_styles import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from PyQt5.QtGui import QCursor, QIcon, QPixmap, QPainter
# ======================================================================================================================
#
#                                               B U T T O N S
#
# ======================================================================================================================
# ----------------------------------------------------------------------------------------------------------------------
#                                              M E N U  B U T T O N
# ----------------------------------------------------------------------------------------------------------------------
class MainMenuButton():
    def __init__(self, text, margin_top, callback_function,
                 border=MENU_BTN_BORDER, border_radius=MENU_BTN_BORDER_RADIUS,
                 font_size=MENU_BTN_FONT_SIZE, font_family=MENU_BTN_FONT_FAMILY, font_weight=MENU_BTN_FONT_WEIGHT,
                 font_color=MENU_BTN_FONT_COLOR, font_hover_color=MENU_BTN_FONT_HOVER_COLOR,
                 background_color=MENU_BTN_BACKGROUND_COLOR, background_hover_color=MENU_BTN_BACKGROUND_HOVER_COLOR):
        self.Button = QPushButton(text)
        self.Button.setCursor(QCursor(QtCore.Qt.PointingHandCursor))
        self.Button.setStyleSheet(
            "*{" +
            f"border: {border};" +
            f"border-radius: {border_radius};" +
            f"font-size: {font_size};" +
            f"font-family: {font_family};" +
            f"font-weight: {font_weight};" +
            f"color: {font_color};" +
            f"background-color: {background_color};" +
            "padding: 5px 0;" +
            f"margin-top: {margin_top};" +
            "}" +
            "*:hover{" +
            f"background: '{background_hover_color}'; color: '{font_hover_color}';" + "}"
        )
        self.Button.clicked.connect(callback_function)

# ----------------------------------------------------------------------------------------------------------------------
#                                              G E N E R I C  B U T T O N
# ----------------------------------------------------------------------------------------------------------------------
class MyButton():
    def __init__(self, text, margin_top, callback_function,
                 border=MENU_BTN_BORDER, border_radius=MENU_BTN_BORDER_RADIUS,
                 font_size=MENU_BTN_FONT_SIZE, font_family=MENU_BTN_FONT_FAMILY, font_weight=MENU_BTN_FONT_WEIGHT,
                 font_color=MENU_BTN_FONT_COLOR, font_hover_color=MENU_BTN_FONT_HOVER_COLOR,
                 background_color=MENU_BTN_BACKGROUND_COLOR, background_hover_color=MENU_BTN_BACKGROUND_HOVER_COLOR):
        self.Button = QPushButton(text)
        self.Button.setCursor(QCursor(QtCore.Qt.PointingHandCursor))
        self.Button.setStyleSheet(
            "*{" +
            f"border: {border};" +
            f"border-radius: {border_radius};" +
            f"font-size: {font_size};" +
            f"font-family: {font_family};" +
            f"font-weight: {font_weight};" +
            f"color: {font_color};" +
            f"background-color: {background_color};" +
            "padding: 5px 0;" +
            f"margin-top: {margin_top};" +
            "}" +
            "*:hover{" +
            f"background: '{background_hover_color}'; color: '{font_hover_color}';" + "}"
        )
        self.Button.clicked.connect(callback_function)
# ======================================================================================================================
#
#                                                   M E N U  B A R
#
# ======================================================================================================================
class MyMenuBar():
    def __init__(self, default_image_path, hover_image_path, callback_function):
        launchfile_menu_layout = QHBoxLayout()
        # Set spacing to zero to avoid visual space columnt:
        launchfile_menu_layout.setSpacing(0)

        menu_button_image_blu = QPixmap(default_image_path).scaledToHeight(30)
        menu_button_image_white = QPixmap(hover_image_path).scaledToHeight(30)
        menu_button_background = QLabel()
        menu_button_background.setStyleSheet(f"background: {MENU_COLOR};")
        menu_button_background.setFixedHeight(50)

        menu_button = QPushButton(menu_button_background)
        menu_button.setFixedSize(menu_button_image_blu.size())
        menu_button.setStyleSheet(
            "*{" +
            f"background-image: url({default_image_path});" +
            "border: 0px;" +
            "}" +
            "*:hover{" +
            f"background-image: url({hover_image_path});" +
            "}"
        )
        menu_button.move(menu_button.parent().pos().x() + 10, menu_button.parent().pos().y() +
                         int(menu_button.parent().frameGeometry().height() / 2 - menu_button.frameGeometry().height() / 2))
        menu_button.clicked.connect(callback_function)
        launchfile_menu_layout.addWidget(menu_button_background)

        launchfile_logo = QLabel()
        menu_logo_image = QPixmap('AppImages/Logo_white.png').scaledToHeight(30)
        launchfile_logo.setPixmap(menu_logo_image)
        launchfile_logo.setAlignment(QtCore.Qt.AlignRight)
        launchfile_logo.setStyleSheet(f'background: {MENU_COLOR}; padding: 10px;')
        launchfile_logo.setFixedHeight(50)
        launchfile_menu_layout.addWidget(launchfile_logo)
        launchfile_menu_layout.setAlignment(QtCore.Qt.AlignTop)
        self.menubar = launchfile_menu_layout

# ======================================================================================================================
#
#                                                   S E C T I O N S
#
# ======================================================================================================================
# ----------------------------------------------------------------------------------------------------------------------
#                                             G E N E R I C  S E C T I O N
# ----------------------------------------------------------------------------------------------------------------------
class GenericSection():
    def __init__(self, offsetx, offsety, text, background_color=MENU_COLOR, text_color='white'):
        # PARAMETERS SECTION:
        self.section_layout = QVBoxLayout()

        # Title:
        section_background = QLabel()
        section_background.setStyleSheet(
            f"background: {background_color};"
        )
        self.section_layout.addWidget(section_background)
        title = QLabel(text, parent=section_background)
        title.setStyleSheet(
            f"color: {text_color};"
        )
        title.move(section_background.pos().x() + offsetx,
                   section_background.pos().y() + offsety
                   )
