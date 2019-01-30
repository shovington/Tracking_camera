import os
import rospkg
import rospy
from rospkg import RosPack

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtCore import QCoreApplication
from PyQt5 import QtCore, QtGui, QtWidgets


class TrackingCamWidget(QtWidgets.QWidget):
    def __init__(self):
        super(TrackingCamWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('tracking_cam_gui'), 'resource', 'tracking_cam.ui')
        loadUi(ui_file, self)

        rp = RosPack()
        self.pkg_path = rp.get_path('tracking_cam_gui')

        self.mode = 0  # Manual mode

        self.manual_btn.clicked[bool].connect(lambda: self.change_mode(0))
        self.disable_btn.clicked[bool].connect(self.disable_motors)
        self.auto_btn.clicked[bool].connect(lambda: self.change_mode(1))
        self.track_btn.clicked[bool].connect(self.track_face)
        self.home_btn.clicked[bool].connect(self.home)

        self.min_yaw.clicked[bool].connect(lambda: self.move("-", "y"))
        self.plus_yaw.clicked[bool].connect(lambda: self.move("+", "y"))
        self.min_pitch.clicked[bool].connect(lambda: self.move("-", "p"))
        self.plus_pitch.clicked[bool].connect(lambda: self.move("+", "p"))
        self.min_roll.clicked[bool].connect(lambda: self.move("-", "r"))
        self.plus_roll.clicked[bool].connect(lambda: self.move("+", "r"))

    def change_mode(self, mode):
        print(mode)
        if mode == 0:
            self.manual_btn.setEnabled(False)
            self.auto_btn.setEnabled(True)
            self.mode = 0
            
        if mode == 1:
            self.manual_btn.setEnabled(True)
            self.auto_btn.setEnabled(False)
            self.mode = 1

    def disable_motors(self):
        print("DISABLE MOTORS")
        pass

    def track_face(self):
        print("TRACK FACE")
        pass

    def home(self):
        print("GO HOME")
        pass

    def move(self, dir, axis):
        print(dir + axis)

