# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'interfaz.ui'
##
## Created by: Qt User Interface Compiler version 6.10.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QPushButton, QSizePolicy,
    QSlider, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(450, 600)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.group_conexion = QGroupBox(self.centralwidget)
        self.group_conexion.setObjectName(u"group_conexion")
        self.horizontalLayout = QHBoxLayout(self.group_conexion)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.ip_input = QLineEdit(self.group_conexion)
        self.ip_input.setObjectName(u"ip_input")

        self.horizontalLayout.addWidget(self.ip_input)

        self.port_input = QLineEdit(self.group_conexion)
        self.port_input.setObjectName(u"port_input")
        self.port_input.setMaximumWidth(80)

        self.horizontalLayout.addWidget(self.port_input)

        self.btn_conectar = QPushButton(self.group_conexion)
        self.btn_conectar.setObjectName(u"btn_conectar")

        self.horizontalLayout.addWidget(self.btn_conectar)

        self.lbl_estado = QLabel(self.group_conexion)
        self.lbl_estado.setObjectName(u"lbl_estado")

        self.horizontalLayout.addWidget(self.lbl_estado)


        self.verticalLayout.addWidget(self.group_conexion)

        self.group_ajustes = QGroupBox(self.centralwidget)
        self.group_ajustes.setObjectName(u"group_ajustes")
        self.formLayout = QFormLayout(self.group_ajustes)
        self.formLayout.setObjectName(u"formLayout")
        self.label_speed = QLabel(self.group_ajustes)
        self.label_speed.setObjectName(u"label_speed")

        self.formLayout.setWidget(0, QFormLayout.ItemRole.LabelRole, self.label_speed)

        self.slider_speed = QSlider(self.group_ajustes)
        self.slider_speed.setObjectName(u"slider_speed")
        self.slider_speed.setMaximum(100)
        self.slider_speed.setValue(70)
        self.slider_speed.setOrientation(Qt.Horizontal)

        self.formLayout.setWidget(0, QFormLayout.ItemRole.FieldRole, self.slider_speed)

        self.lbl_speed_val = QLabel(self.group_ajustes)
        self.lbl_speed_val.setObjectName(u"lbl_speed_val")

        self.formLayout.setWidget(1, QFormLayout.ItemRole.LabelRole, self.lbl_speed_val)

        self.label_lights = QLabel(self.group_ajustes)
        self.label_lights.setObjectName(u"label_lights")

        self.formLayout.setWidget(2, QFormLayout.ItemRole.LabelRole, self.label_lights)

        self.lights_layout = QHBoxLayout()
        self.lights_layout.setObjectName(u"lights_layout")
        self.check_stop = QCheckBox(self.group_ajustes)
        self.check_stop.setObjectName(u"check_stop")

        self.lights_layout.addWidget(self.check_stop)

        self.combo_turn = QComboBox(self.group_ajustes)
        self.combo_turn.addItem("")
        self.combo_turn.addItem("")
        self.combo_turn.addItem("")
        self.combo_turn.addItem("")
        self.combo_turn.setObjectName(u"combo_turn")

        self.lights_layout.addWidget(self.combo_turn)


        self.formLayout.setLayout(2, QFormLayout.ItemRole.FieldRole, self.lights_layout)


        self.verticalLayout.addWidget(self.group_ajustes)

        self.group_control = QGroupBox(self.centralwidget)
        self.group_control.setObjectName(u"group_control")
        self.gridLayout = QGridLayout(self.group_control)
        self.gridLayout.setObjectName(u"gridLayout")
        self.btn_w = QPushButton(self.group_control)
        self.btn_w.setObjectName(u"btn_w")
        self.btn_w.setMinimumSize(QSize(80, 80))

        self.gridLayout.addWidget(self.btn_w, 0, 1, 1, 1)

        self.btn_a = QPushButton(self.group_control)
        self.btn_a.setObjectName(u"btn_a")
        self.btn_a.setMinimumSize(QSize(80, 80))

        self.gridLayout.addWidget(self.btn_a, 1, 0, 1, 1)

        self.btn_s = QPushButton(self.group_control)
        self.btn_s.setObjectName(u"btn_s")
        self.btn_s.setMinimumSize(QSize(80, 80))

        self.gridLayout.addWidget(self.btn_s, 1, 1, 1, 1)

        self.btn_d = QPushButton(self.group_control)
        self.btn_d.setObjectName(u"btn_d")
        self.btn_d.setMinimumSize(QSize(80, 80))

        self.gridLayout.addWidget(self.btn_d, 1, 2, 1, 1)


        self.verticalLayout.addWidget(self.group_control)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Gamepad - Robot Ackermann", None))
        self.group_conexion.setTitle(QCoreApplication.translate("MainWindow", u"1. Conexi\u00f3n", None))
        self.ip_input.setText(QCoreApplication.translate("MainWindow", u"127.0.0.1", None))
        self.port_input.setText(QCoreApplication.translate("MainWindow", u"65432", None))
        self.btn_conectar.setText(QCoreApplication.translate("MainWindow", u"Conectar", None))
        self.lbl_estado.setText(QCoreApplication.translate("MainWindow", u"Desconectado", None))
        self.lbl_estado.setStyleSheet(QCoreApplication.translate("MainWindow", u"color: red; font-weight: bold;", None))
        self.group_ajustes.setTitle(QCoreApplication.translate("MainWindow", u"2. Ajustes de Velocidad y Luces", None))
        self.label_speed.setText(QCoreApplication.translate("MainWindow", u"L\u00edmite de Velocidad:", None))
        self.lbl_speed_val.setText(QCoreApplication.translate("MainWindow", u"70%", None))
        self.label_lights.setText(QCoreApplication.translate("MainWindow", u"Luces Manuales:", None))
        self.check_stop.setText(QCoreApplication.translate("MainWindow", u"Freno (Stop)", None))
        self.combo_turn.setItemText(0, QCoreApplication.translate("MainWindow", u"Apagado (0)", None))
        self.combo_turn.setItemText(1, QCoreApplication.translate("MainWindow", u"Derecha (1)", None))
        self.combo_turn.setItemText(2, QCoreApplication.translate("MainWindow", u"Izquierda (2)", None))
        self.combo_turn.setItemText(3, QCoreApplication.translate("MainWindow", u"Intermitentes (3)", None))

        self.group_control.setTitle(QCoreApplication.translate("MainWindow", u"3. Control de Movimiento (Teclado o Clic)", None))
        self.btn_w.setText(QCoreApplication.translate("MainWindow", u"W\n"
"(Adelante)", None))
        self.btn_a.setText(QCoreApplication.translate("MainWindow", u"A\n"
"(Izquierda)", None))
        self.btn_s.setText(QCoreApplication.translate("MainWindow", u"S\n"
"(Atr\u00e1s)", None))
        self.btn_d.setText(QCoreApplication.translate("MainWindow", u"D\n"
"(Derecha)", None))
    # retranslateUi

