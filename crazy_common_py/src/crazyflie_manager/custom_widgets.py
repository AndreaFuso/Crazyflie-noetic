from app_styles import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from PyQt5.QtGui import QCursor, QIcon, QPixmap
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

class MenuButton():
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
        self.Button.setIcon(QIcon(QPixmap('AppImages/Menu_icon_white.png').scaledToHeight(30)))

# ======================================================================================================================
#
#                                                   M E N U  B A R
#
# ======================================================================================================================
class MenuBar():
    def __init__(self):
        pass