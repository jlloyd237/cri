# -*- coding: utf-8 -*-
"""Simple robot jogger app based on Common Robot Interface.
"""

import os
import sys
import ipaddress
import logging

from PyQt5.QtCore import Qt, pyqtSignal, pyqtProperty
from PyQt5.QtWidgets import QApplication, QDialog, QStyleFactory, QToolTip, \
    QTabWidget, QSizePolicy, QWidget, QHBoxLayout, QVBoxLayout, QScrollBar, \
    QGridLayout, QLabel, QLineEdit, QPushButton, QComboBox, QMessageBox, \
    QGroupBox, QErrorMessage
from PyQt5.QtGui import QIcon, QFont

from cri.robot import SyncRobot
from cri.controller import ABBController, RTDEController, PyfrankaController

# Uncomment for testing/debugging
# from dummy_robot import DummySyncRobot as SyncRobot
# from dummy_robot import DummyABBController as ABBController, \
#     DummyRTDEController as RTDEController, DummyFrankxController as FrankxController, \
#     DummyPyfrankaController as PyfrankaController

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def isValidNumber(num, bottom=None, top=None):
    try:
        num = float(num)
        if bottom is not None and num < bottom:
            raise ValueError
        if top is not None and num > top:
            raise ValueError
    except ValueError:
        return False
    else:
        return True


def isValidIPAddress(ip):
    try:
        ipaddress.ip_address(ip)
    except ValueError:
        return False
    else:
        return True    


def isValidPortNumber(port):
    try:
        port = int(port)
        if not 1 <= port <= 65535:
            raise ValueError
    except ValueError:
        return False
    else:
        return True


class JoggerDialog(QDialog):
    """Robot jogger dialog allows user to connect/disconnect to/from robot,
    modify its settings (axes, linear speed, angular speed, blend radius,
    tcp and coordinate frame), and control its pose and joint angles.
    """
    
    class TabIndex:
        SETTINGS = 0
        POSE = 1
        JOINTS = 2
        
    ROBOTS = ("abb", "ur", "franka")

    def __init__(self, parent=None):
        super().__init__(parent)

        self.robot = None

        QApplication.setStyle(QStyleFactory.create("Fusion"))  
        QToolTip.setFont(QFont("SansSerif", 10))  
        
        self.setFixedSize(800, 450)
        self.setWindowTitle("Robot Jogger")
        self.setWindowIcon(QIcon(os.path.join(os.path.dirname(__file__), "robot.png")))         
        self.setToolTip("Robot jogger based on Common Robot Interface")
     
        self.robotLabel = QLabel("robot:")
        self.robotLabel.setFixedWidth(50)
        self.robotLabel.setFixedHeight(20)
        self.robotLabel.setAlignment(Qt.AlignRight|Qt.AlignVCenter)

        self.robotComboBox = QComboBox()
        self.robotComboBox.addItems(self.ROBOTS)
        self.robotComboBox.setCurrentText("abb")
        self.robotComboBox.setFixedWidth(70)
        self.robotComboBox.setFixedHeight(20)
        self.robotComboBox.setEnabled(True)
        
        self.ipLabel = QLabel("ip:")
        self.ipLabel.setFixedWidth(20)
        self.ipLabel.setFixedHeight(20)
        self.ipLabel.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
        
        self.ipEditBox = LineEdit()
        self.ipEditBox.setFixedHeight(20)
        self.ipEditBox.setEnabled(True)
        
        self.portLabel = QLabel("port:")
        self.portLabel.setFixedWidth(40)
        self.portLabel.setFixedHeight(20)
        self.portLabel.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
        
        self.portEditBox = LineEdit()
        self.portEditBox.setFixedWidth(60)
        self.portEditBox.setFixedHeight(20)
        self.portEditBox.setEnabled(True)
        
        self.connectPushButton = QPushButton("&Connect")
        self.connectPushButton.setFixedHeight(20)
        self.connectPushButton.setDefault(False)
        self.connectPushButton.setAutoDefault(True)
        self.connectPushButton.setEnabled(True)
        
        self.disconnectPushButton = QPushButton("&Disconnect")
        self.disconnectPushButton.setFixedHeight(20)       
        self.disconnectPushButton.setDefault(False)
        self.disconnectPushButton.setAutoDefault(False)
        self.disconnectPushButton.setEnabled(False)

        topLayout = QGridLayout()
        topLayout.setHorizontalSpacing(20)
        topLayout.setVerticalSpacing(20)
        topLayout.setContentsMargins(10, 10, 10, 10)
        topLayout.addWidget(self.robotLabel, 0, 0)
        topLayout.addWidget(self.robotComboBox, 0, 1)
        topLayout.addWidget(self.ipLabel, 0, 2)
        topLayout.addWidget(self.ipEditBox, 0, 3)
        topLayout.addWidget(self.portLabel, 0, 4)
        topLayout.addWidget(self.portEditBox, 0, 5)
        topLayout.addWidget(self.connectPushButton, 0, 6)        
        topLayout.addWidget(self.disconnectPushButton, 1, 6)

        robot = self.robotComboBox.currentText()
        self.createTabWidget(robot)
        self.tabWidget.setEnabled(False)

        mainLayout = QVBoxLayout()
        mainLayout.addLayout(topLayout)        
        mainLayout.addWidget(self.tabWidget)
        
        self.setLayout(mainLayout)

        self.robotComboBox.currentIndexChanged[str].connect(self.onRobotChanged)
        self.ipEditBox.textEditingFinished.connect(self.onIPAddressChanged)
        self.portEditBox.textEditingFinished.connect(self.onPortChanged)
        self.connectPushButton.clicked.connect(self.onConnect)
        self.disconnectPushButton.clicked.connect(self.onDisconnect)             

        self.axesComboBox.currentIndexChanged.connect(self.onAxesChanged)
        self.linearSpeedEditBox.textEditingFinished.connect(self.onLinearSpeedChanged)
        self.angularSpeedEditBox.textEditingFinished.connect(self.onAngularSpeedChanged)
        self.blendRadiusEditBox.textEditingFinished.connect(self.onBlendRadiusChanged)
        
        self.tcpEditWidget.editingFinished.connect(self.onTCPChanged)
        self.coordFrameEditWidget.editingFinished.connect(self.onCoordFrameChanged)
       
        self.poseControlWidget.valueChanged.connect(self.onPoseChanged)
        
        self.tabWidget.currentChanged.connect(self.onTabChanged) 


    def createTabWidget(self, robot):
        self.tabWidget = QTabWidget()
        self.tabWidget.setSizePolicy(QSizePolicy.Preferred,
                                         QSizePolicy.Ignored)

        settingsTab = QWidget()  
        self.createControlSettingsWidget()
        settingsTabLayout = QHBoxLayout()
        settingsTabLayout.setContentsMargins(5, 5, 5, 5)
        settingsTabLayout.addWidget(self.controlSettings)
        settingsTab.setLayout(settingsTabLayout)

        poseTab = QWidget()        
        self.poseControlWidget = MultiSliderControlWidget(
            names=("x", "y", "z", "alpha", "beta", "gamma"),
            units=("mm",) * 3 + ("°",) * 3,
            ranges=((-1000, 1000),) * 3 + ((-360, 360),) * 3,
            values=(0,) * 6,
            )
        poseTabLayout = QHBoxLayout()
        poseTabLayout.setContentsMargins(5, 5, 5, 5)
        poseTabLayout.addWidget(self.poseControlWidget)        
        poseTab.setLayout(poseTabLayout)

        jointsTab = QWidget()   # widget content depends on robot type

        self.tabWidget.addTab(settingsTab, "&Settings" )
        self.tabWidget.addTab(poseTab, "&Pose")
        self.tabWidget.addTab(jointsTab, "&Joints")

    def createControlSettingsWidget(self):
        self.controlSettings = QWidget()
        
        self.minLinearSpeed = 0.001
        self.maxLinearSpeed = 1000
        self.minAngularSpeed = 0.001
        self.maxAngularSpeed = 1000
        self.minBlendRadius = 0
        self.maxBlendRadius = 1000

        miscSettingsGroupBox = QGroupBox(title="Misc:")

        self.axesLabel = QLabel("axes:")
        self.axesLabel.setFixedWidth(50)
        self.axesLabel.setFixedHeight(20)

        self.axesComboBox = QComboBox()
        self.axesComboBox.addItems(SyncRobot.EULER_AXES)
        self.axesComboBox.setCurrentText("rxyz")
        self.axesComboBox.setFixedHeight(20)
        
        self.axisUnitsLabel = QLabel()
        self.axisUnitsLabel.setFixedWidth(40)
        self.axisUnitsLabel.setFixedHeight(20)        

        self.linearSpeedLabel = QLabel("linear\nspeed:")
        self.linearSpeedLabel.setFixedWidth(50)
        self.linearSpeedLabel.setFixedHeight(40)

        self.linearSpeedEditBox = LineEdit()
        self.linearSpeedEditBox.setText("{:.1f}".format(0))
        self.linearSpeedEditBox.setFixedHeight(20)
        self.linearSpeedEditBox.setAlignment(Qt.AlignRight|Qt.AlignVCenter)

        self.linearSpeedUnitsLabel = QLabel("mm/s")
        self.linearSpeedUnitsLabel.setFixedWidth(40)
        self.linearSpeedUnitsLabel.setFixedHeight(20)

        self.angularSpeedLabel = QLabel("angular\nspeed:")
        self.angularSpeedLabel.setFixedWidth(50)
        self.angularSpeedLabel.setFixedHeight(40)             

        self.angularSpeedEditBox = LineEdit()
        self.angularSpeedEditBox.setText("{:.1f}".format(0))
        self.angularSpeedEditBox.setFixedHeight(20)
        self.angularSpeedEditBox.setAlignment(Qt.AlignRight|Qt.AlignVCenter)

        self.angularSpeedUnitsLabel = QLabel("°/s")
        self.angularSpeedUnitsLabel.setFixedWidth(40)
        self.angularSpeedUnitsLabel.setFixedHeight(20)
        
        self.blendRadiusLabel = QLabel("blend\nradius:")
        self.blendRadiusLabel.setFixedWidth(50)
        self.blendRadiusLabel.setFixedHeight(40)             

        self.blendRadiusEditBox = LineEdit()
        self.blendRadiusEditBox.setText("{:.1f}".format(0))
        self.blendRadiusEditBox.setFixedHeight(20)
        self.blendRadiusEditBox.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
        self.blendRadiusEditBox.setEnabled(False)

        self.blendRadiusUnitsLabel = QLabel("mm")
        self.blendRadiusUnitsLabel.setFixedWidth(40)
        self.blendRadiusUnitsLabel.setFixedHeight(20)   

        miscSettingsLayout = QGridLayout()
        miscSettingsLayout.setHorizontalSpacing(15)
        miscSettingsLayout.setVerticalSpacing(5)
        miscSettingsLayout.addWidget(self.axesLabel, 0, 0)
        miscSettingsLayout.addWidget(self.axesComboBox, 0, 1)
        miscSettingsLayout.addWidget(self.linearSpeedLabel, 1, 0)
        miscSettingsLayout.addWidget(self.linearSpeedEditBox, 1, 1)
        miscSettingsLayout.addWidget(self.linearSpeedUnitsLabel, 1, 2)        
        miscSettingsLayout.addWidget(self.angularSpeedLabel, 2, 0)
        miscSettingsLayout.addWidget(self.angularSpeedEditBox, 2, 1)
        miscSettingsLayout.addWidget(self.angularSpeedUnitsLabel, 2, 2)
        miscSettingsLayout.addWidget(self.blendRadiusLabel, 3, 0)
        miscSettingsLayout.addWidget(self.blendRadiusEditBox, 3, 1)
        miscSettingsLayout.addWidget(self.blendRadiusUnitsLabel, 3, 2)        
        miscSettingsGroupBox.setLayout(miscSettingsLayout)
         
        self.tcpEditWidget = MultiNumberEditWidget(
            names=("x", "y", "z", "alpha", "beta", "gamma"),
            units=("mm",) * 3 + ("°",) * 3,
            ranges=((-1000, 1000),) * 3 + ((-360, 360),) * 3,
            values=(0,) * 6,
            )

        tcpGroupBox = QGroupBox(title="TCP:")
        tcpLayout = QVBoxLayout()
        tcpLayout.addWidget(self.tcpEditWidget)
        tcpGroupBox.setLayout(tcpLayout)
           
        self.coordFrameEditWidget = MultiNumberEditWidget(
            names=("x", "y", "z", "alpha", "beta", "gamma"),
            units=("mm",) * 3 + ("°",) * 3,
            ranges=((-1000, 1000),) * 3 + ((-360, 360),) * 3,
            values=(0,) * 6,
            )
        
        coordFrameGroupBox = QGroupBox(title="Coordinate Frame:")
        coordFrameLayout = QVBoxLayout()
        coordFrameLayout.addWidget(self.coordFrameEditWidget)
        coordFrameGroupBox.setLayout(coordFrameLayout)        

        controlSettingsLayout = QGridLayout()
        controlSettingsLayout.setHorizontalSpacing(15)
        controlSettingsLayout.setVerticalSpacing(5)
        controlSettingsLayout.addWidget(miscSettingsGroupBox, 0, 0)
        controlSettingsLayout.addWidget(tcpGroupBox, 0, 1)
        controlSettingsLayout.addWidget(coordFrameGroupBox, 0, 2)
        self.controlSettings.setLayout(controlSettingsLayout)        

    def closeEvent(self, event):      
        reply = QMessageBox.question(self, "Message",
            "Are you sure you want to quit?", QMessageBox.Yes | 
            QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            if self.robot is not None:
                self.robot.close()
            event.accept()
        else:
            event.ignore()

    def onRobotChanged(self, value):
        logger.debug("JoggerDialog.onRobotChanged(value={})".format(
                value))
        if value == "franka":
            # No need to specify port for Franka arm
            self.portEditBox.setText("")
            self.portEditBox.setEnabled(False)
        elif value == "ur":
            # UR controller always listens on port 30004
            self.portEditBox.setText(str(30004))
            self.portEditBox.setEnabled(False)
        else:
            self.portEditBox.setEnabled(True)            

    def onIPAddressChanged(self):
        ip = self.sender().text()
        logger.debug("JoggerDialog.onIPAddressChanged(text={})".format(ip))       
        if not isValidIPAddress(ip):
            msg = QErrorMessage(self)
            msg.showMessage("Invalid IP address")
            msg.exec_()
            self.sender().setFocus()

    def onPortChanged(self):
        port = self.sender().text()
        logger.debug("JoggerDialog.onPortChanged(text={})".format(port))       
        if not isValidPortNumber(port):
            msg = QErrorMessage(self)
            msg.showMessage("Invalid port number")
            msg.exec_()
            self.sender().setFocus()
    
    def onConnect(self):
        robot = self.robotComboBox.currentText()
        ip = self.ipEditBox.text()
        port = self.portEditBox.text()
        logger.debug("JoggerDialog.onConnect(robot={}, ip={}, port={})".format(
                robot, ip, port))

        if robot in ("abb", 'ur') and (not isValidIPAddress(ip) or not isValidPortNumber(port)):
            msg = QErrorMessage(self)
            msg.showMessage("Please specify valid IP address and port number")
            msg.exec_()
            self.sender().setFocus()
            return

        if robot == "franka" and not isValidIPAddress(ip):   # no need to specify port for Franka arm
            msg = QErrorMessage(self)
            msg.showMessage("Please specify valid IP address")
            msg.exec_()
            self.sender().setFocus()
            return

        try:
            if robot == "abb":
                port = int(port)
                self.robot = SyncRobot(ABBController(ip, port))
            elif robot == "ur":
                self.robot = SyncRobot(RTDEController(ip))
            elif robot == "franka":
                self.robot = SyncRobot(PyfrankaController(ip))
        except:
            msg = QErrorMessage(self)
            msg.showMessage("Failed to connect to server")
            msg.exec_()
        else:
            self.robotComboBox.setEnabled(False)
            self.ipEditBox.setEnabled(False)
            self.portEditBox.setEnabled(False)
            self.connectPushButton.setEnabled(False)
            self.disconnectPushButton.setEnabled(True)

            self.axesComboBox.setCurrentText(self.robot.axes)

            if robot == "franka":
                # Disable settings that don't apply to Franka arm
                self.linearSpeedEditBox.setText("n/a")
                self.angularSpeedEditBox.setText("n/a")
                self.blendRadiusEditBox.setText("n/a")
                self.linearSpeedEditBox.setEnabled(False)
                self.blendRadiusEditBox.setEnabled(False)
                self.angularSpeedEditBox.setEnabled(False)
            else:
                self.linearSpeedEditBox.setText("{:.1f}".format(self.robot.linear_speed))
                self.angularSpeedEditBox.setText("{:.1f}".format(self.robot.angular_speed))
                self.blendRadiusEditBox.setText("{:.1f}".format(self.robot.blend_radius))
                self.linearSpeedEditBox.setEnabled(True)
                self.blendRadiusEditBox.setEnabled(True)
                self.angularSpeedEditBox.setEnabled(True)

            self.tcpEditWidget.value = self.robot.tcp
            self.coordFrameEditWidget.value = self.robot.coord_frame

            # Set up joint angles tab - need to do this dynamically now because layout depends on
            # whether ABB/UR5 robot is selected (6 joints), or Franka robot is selected (7 joints)
            jointsTab = QWidget()
            n_joints = 7 if robot == "franka" else 6
            self.jointControlWidget = MultiSliderControlWidget(
                names=["joint {}".format(i+1) for i in range(n_joints)],
                units=("°",) * n_joints,
                ranges=((-360, 360),) * n_joints,
                values=(0,) * n_joints,
            )
            self.jointControlWidget.valueChanged.connect(self.onJointAnglesChanged)
            jointsTabLayout = QHBoxLayout()
            jointsTabLayout.setContentsMargins(5, 5, 5, 5)
            jointsTabLayout.addWidget(self.jointControlWidget)
            jointsTab.setLayout(jointsTabLayout)
            self.tabWidget.removeTab(2)
            self.tabWidget.addTab(jointsTab, "&Joints")

            self.tabWidget.setCurrentIndex(self.TabIndex.SETTINGS)
            self.tabWidget.setEnabled(True)

    def onDisconnect(self):
        logger.debug("JoggerDialog.onDisconnect")         
        self.robotComboBox.setEnabled(True)
        self.ipEditBox.setEnabled(True)
        robot = self.robotComboBox.currentText()
        if robot == "abb":
            self.portEditBox.setEnabled(True)
        elif robot in ("ur", "franka"):
            self.portEditBox.setEnabled(False)
        self.connectPushButton.setEnabled(True)
        self.disconnectPushButton.setEnabled(False)
        self.tabWidget.setCurrentIndex(self.TabIndex.SETTINGS)
        self.tabWidget.setEnabled(False)
        self.robot.close()
        self.robot = None
            
    def onAxesChanged(self):
        axes = self.sender().currentText()
        logger.debug("JoggerDialog.onAxesChanged(axes={})".format(axes))        
        self.robot.axes = axes
        self.tcpEditWidget.value = self.robot.tcp
        self.coordFrameEditWidget.value = self.robot.coord_frame
    
    def onLinearSpeedChanged(self):
        linearSpeed = self.sender().text()
        logger.debug("JoggerDialog.onLinearSpeedChanged(speed={})".format(
                linearSpeed))        
        if isValidNumber(linearSpeed, self.minLinearSpeed, self.maxLinearSpeed):
            linearSpeed = float(linearSpeed)
            self.sender().setText("{:.1f}".format(linearSpeed))
            self.robot.linear_speed = linearSpeed
        else:            
            msg = QErrorMessage(self)
            msg.showMessage("Invalid or out of range [{}, {}] number".format(
                    self.minLinearSpeed, self.maxLinearSpeed))
            msg.exec_()
            self.sender().setText("{:.1f}".format(0))
            self.sender().setFocus()
    
    def onAngularSpeedChanged(self):
        angularSpeed = self.sender().text()
        logger.debug("JoggerDialog.onAngularSpeedChanged(speed={})".format(
                angularSpeed))
        if isValidNumber(angularSpeed, self.minAngularSpeed, self.maxAngularSpeed):
            angularSpeed = float(angularSpeed)
            self.sender().setText("{:.1f}".format(angularSpeed))
            self.robot.angular_speed = angularSpeed
        else:            
            msg = QErrorMessage(self)
            msg.showMessage("Invalid or out of range [{}, {}] number".format(
                    self.minAngularSpeed, self.maxAngularSpeed))
            msg.exec_()
            self.sender().setText("{:.1f}".format(0))
            self.sender().setFocus()
           
    def onBlendRadiusChanged(self):
        blendRadius = self.blendRadiusEditBox.text()
        logger.debug("JoggerDialog.onBlendRadiusChanged(radius={})".format(
                blendRadius))
        if isValidNumber(blendRadius, self.minBlendRadius, self.maxBlendRadius):
            blendRadius = float(blendRadius)
            self.sender().setText("{:.1f}".format(blendRadius))
            self.robot.blend_radius = blendRadius
        else:            
            msg = QErrorMessage(self)
            msg.showMessage("Invalid or out of range [{}, {}] number".format(
                    self.minBlendRadius, self.maxBlendRadius))
            msg.exec_()
            self.sender().setText("{:.1f}".format(0))
            self.sender().setFocus()   
    
    def onTCPChanged(self):
        logger.debug("JoggerDialog.onTCPChanged(tcp={})".format(
                self.tcpEditWidget.value))         
        self.robot.tcp = self.tcpEditWidget.value
    
    def onCoordFrameChanged(self):
        logger.debug("JoggerDialog.onCoordFrameChanged(frame={})".format(
                self.coordFrameEditWidget.value))          
        self.robot.coord_frame = self.coordFrameEditWidget.value
    
    def onPoseChanged(self):
        logger.debug("JoggerDialog.onPoseChanged(pose={})".format(
                self.poseControlWidget.value))          
        self.robot.move_linear(self.poseControlWidget.value)
    
    def onJointAnglesChanged(self):
        logger.debug("JoggerDialog.onJointAnglesChanged(angles={})".format(
                self.jointControlWidget.value))  
        self.robot.move_joints(self.jointControlWidget.value)
    
    def onTabChanged(self, index):
        logger.debug("JoggerDialog.onTabChanged(index={}, text={})".format(
                index, self.tabWidget.tabText(index)))    
        if index == self.TabIndex.POSE:
            self.poseControlWidget.value = self.robot.pose
        elif index == self.TabIndex.JOINTS:
            self.jointControlWidget.value = self.robot.joint_angles


class MultiSliderControlWidget(QWidget):
    """Multiple slider control widget with label, slider, edit box, units and
    validation.
    """
    valueChanged = pyqtSignal()

    def __init__(self, names, units, ranges, values, parent=None):
        super().__init__(parent)
       
        layout = QVBoxLayout()
        layout.setSpacing(5)
        
        self.sliders = []
        for i in range(len(names)):
            slider = SliderControlWidget(names[i], units[i], ranges[i], values[i])
            self.sliders.append(slider)
            slider.valueChanged.connect(self.onValueChanged)
            layout.addWidget(slider)
            
        self.setLayout(layout)

    @pyqtProperty(list)
    def value(self):
        return [s.value for s in self.sliders]
    
    @value.setter
    def value(self, val):
        for i, v in enumerate(val):
            self.sliders[i].value = v

    def onValueChanged(self):             
        self.valueChanged.emit()


class SliderControlWidget(QWidget):
    """Slider control widget with label, slider, edit box, units and validation.
    """
    valueChanged = pyqtSignal()
    
    def __init__(self, name, units, range_, value, parent=None):
        super().__init__(parent)
        
        layout = QHBoxLayout()
        layout.setSpacing(20)

        self.minValue = range_[0]
        self.maxValue = range_[1]

        self.nameLabel = QLabel(name + ":")
        self.nameLabel.setFixedWidth(60)
        self.nameLabel.setFixedHeight(20)

        self.scrollBar = ScrollBar(Qt.Horizontal)
        self.scrollBar.setMinimum(self.minValue)
        self.scrollBar.setMaximum(self.maxValue)
        self.scrollBar.setValue(value)
        self.scrollBar.setFixedHeight(20)
        self.scrollBar.setTracking(False)
                
        self.editBox = LineEdit()
        self.editBox.setText("{:.1f}".format(value))
        self.editBox.setFixedWidth(60)
        self.editBox.setFixedHeight(20)
        self.editBox.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
        
        self.unitsLabel = QLabel(units)
        self.unitsLabel.setFixedWidth(30)
        self.unitsLabel.setFixedHeight(20)

        self.scrollBar.actionValueChanged.connect(self.onValueChanged)        
        self.editBox.textEditingFinished.connect(self.onValueChanged)
        
        layout.addWidget(self.nameLabel)
        layout.addWidget(self.scrollBar)
        layout.addWidget(self.editBox)
        layout.addWidget(self.unitsLabel)
        
        self.setLayout(layout)
        
    @pyqtProperty(float)
    def value(self):
        return float(self.editBox.text())

    @value.setter
    def value(self, value):
        self.editBox.setText("{:.1f}".format(value))
        self.scrollBar.setValue(int(float(value)))

    def onValueChanged(self):
        if isinstance(self.sender(), ScrollBar):
            value = self.sender().value()
            self.editBox.setText("{:.1f}".format(value))
            self.editBox.setFocus()
            self.valueChanged.emit()
        else:   # sender = LineEdit
            value = self.sender().text()
            if isValidNumber(value, self.minValue, self.maxValue):
                value = float(value)
                self.scrollBar.setValue(value)
                self.editBox.setText("{:.1f}".format(value))
                self.valueChanged.emit()
            else:            
                msg = QErrorMessage(self)
                msg.showMessage("Invalid or out of range [{}, {}] number".format(
                        self.minValue, self.maxValue))
                msg.exec_()
                self.sender().setText("{:.1f}".format(self.scrollBar.value()))
                self.sender().setFocus()            


class MultiNumberEditWidget(QWidget):
    """Multiple number edit class with label, edit box, units and validation.
    """ 
    editingFinished = pyqtSignal()

    def __init__(self, names, units, ranges, values, parent=None):
        super().__init__(parent)
       
        layout = QVBoxLayout()
        layout.setSpacing(5)
        
        self.widgets = []
        for i in range(len(names)):
            widget = NumberEditWidget(names[i], units[i], ranges[i], values[i])
            self.widgets.append(widget)
            widget.editingFinished.connect(self.onEditingFinished)
            layout.addWidget(widget)
            
        self.setLayout(layout)

    @pyqtProperty(list)
    def value(self):
        return [w.value for w in self.widgets]
    
    @value.setter
    def value(self, value):
        for i, v in enumerate(value):
            self.widgets[i].value = v

    def onEditingFinished(self):             
        self.editingFinished.emit()


class NumberEditWidget(QWidget):
    """Number edit widget with label, edit box, units and validation.
    """ 
    editingFinished = pyqtSignal()
    
    def __init__(self, name, units, range_, value, parent=None):
        super().__init__(parent)    

        layout = QHBoxLayout()
        layout.setSpacing(20)

        self.minValue = range_[0]
        self.maxValue = range_[1]

        self.nameLabel = QLabel(name + ":")
        self.nameLabel.setFixedWidth(50)
        self.nameLabel.setFixedHeight(20)
                
        self.editBox = LineEdit()
        self.editBox.setText("{:.1f}".format(value))
        self.editBox.setFixedHeight(20)
        self.editBox.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
        
        self.unitsLabel = QLabel(units)
        self.unitsLabel.setFixedWidth(30)
        self.unitsLabel.setFixedHeight(20)
       
        self.editBox.textEditingFinished.connect(self.onTextEditingFinished)
        
        layout.addWidget(self.nameLabel)
        layout.addWidget(self.editBox)
        layout.addWidget(self.unitsLabel)
        
        self.setLayout(layout)

    @pyqtProperty(list)
    def value(self):
        return float(self.editBox.text())
    
    @value.setter
    def value(self, value):
        self.editBox.setText("{:.1f}".format(value))

    def onTextEditingFinished(self):
        value = self.sender().text()
        if isValidNumber(value, self.minValue, self.maxValue):
            value = float(value)
            self.editBox.setText("{:.1f}".format(value))
            self.editingFinished.emit()
        else:            
            msg = QErrorMessage(self)
            msg.showMessage("Invalid or out of range [{}, {}] number".format(
                    self.minValue, self.maxValue))
            msg.exec_()
            self.sender().setText("{:.1f}".format(0))
            self.sender().setFocus()


class LineEdit(QLineEdit):
    """Line edit widget signals completion of non-programmatic text editing.
    """ 
    textEditingFinished = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.editing = False
     
        self.textEdited[str].connect(self.onTextEdited)
        self.editingFinished.connect(self.onEditingFinished)
       
    def onTextEdited(self, text):
        self.editing = True
        
    def onEditingFinished(self):       
        if self.editing:
            self.editing = False
            self.textEditingFinished.emit()
            
            
class ScrollBar(QScrollBar):
    """Scroll bar widget signals non-programmatic change in scroll bar value.
    """
    actionValueChanged = pyqtSignal()
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sliderAction = False
        
        self.actionTriggered[int].connect(self.onActionTriggered)
        self.valueChanged[int].connect(self.onValueChanged)        
            
    def onActionTriggered(self, action):
        self.sliderAction = True

    def onValueChanged(self, value):
        if self.sliderAction:
            self.sliderAction = False
            self.actionValueChanged.emit()

        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    jogger = JoggerDialog()
    jogger.show()
    sys.exit(app.exec_())
