import os
import rospkg
import rospy
from rospkg import RosPack

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtCore import QCoreApplication
from PyQt5 import QtCore, QtGui, QtWidgets

from std_msgs.msg import Float32, Bool, Int32

CMD = {"MIN_YAW": 0,
       "PLUS_YAW": 1,
       "MIN_PITCH": 2,
       "PLUS_PITCH": 3,
       "MIN_ROLL": 4,
       "PLUS_ROLL": 5,
       "HOME":6,
       "ENABLE":7}


class TrackingCamWidget(QtWidgets.QWidget):
    def __init__(self):
        super(TrackingCamWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('tracking_cam_gui'), 'resource', 'tracking_cam.ui')
        loadUi(ui_file, self)

        rp = RosPack()
        self.pkg_path = rp.get_path('tracking_cam_gui')

        self.mode = 0  # Manual mode

        self.manual_btn.clicked[bool].connect(lambda: self.change_mode(0))
        self.disable_btn.clicked[bool].connect(lambda: self.move(CMD["ENABLE"]))
        self.auto_btn.clicked[bool].connect(lambda: self.change_mode(1))
        self.track_btn.clicked[bool].connect(self.track_face)
        self.home_btn.clicked[bool].connect(lambda: self.move(CMD["HOME"]))

        self.min_yaw.clicked[bool].connect(lambda: self.move(CMD[MIN_YAW]))
        self.plus_yaw.clicked[bool].connect(lambda: self.move(CMD[PLUS_YAW]))
        self.min_pitch.clicked[bool].connect(lambda: self.move(CMD[MIN_PITCH]))
        self.plus_pitch.clicked[bool].connect(lambda: self.move(CMD[PLUS_PITCH]))
        self.min_roll.clicked[bool].connect(lambda: self.move(CMD[MIN_ROLL]))
        self.plus_roll.clicked[bool].connect(lambda: self.move(CMD[PLUS_ROLL]))

        self.cmd_pub = rospy.Publisher("cmd_manual", Int32, queue_size=10)


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
        self.cmd_pub(CMD["ENABLE"])

    def track_face(self):
        print("TRACK FACE")
        pass

    def move(self, dir):
        self.cmd_pub.publish(dir)

