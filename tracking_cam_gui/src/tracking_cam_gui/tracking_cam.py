import os
import rospkg
import rospy
from rospkg import RosPack

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtCore import QCoreApplication
from PyQt5 import QtCore, QtGui, QtWidgets

from std_msgs.msg import Float32, Bool, Int32
from constants import * 
from dynamixel_workbench_msgs.msg import *
from std_msgs.msg import Float32, Bool, Int32, Int32MultiArray


class TrackingCamWidget(QtWidgets.QWidget):
    def __init__(self):
        super(TrackingCamWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('tracking_cam_gui'), 'resource', 'tracking_cam.ui')
        loadUi(ui_file, self)

        rp = RosPack()
        self.pkg_path = rp.get_path('tracking_cam_gui')

        self.mode = MANUAL  # Manual mode

        self.disable_btn.clicked[bool].connect(self.enable_motors)
        self.auto_btn.clicked[bool].connect(lambda: self.change_mode(AUTO))
        self.track_btn.clicked[bool].connect(self.track_face)
        self.home_btn.clicked[bool].connect(lambda: self.move(CMD["HOME"]))

        #self.min_yaw.clicked[bool].connect(lambda: self.move(CMD["MIN_YAW"]))
        #self.plus_yaw.clicked[bool].connect(lambda: self.move(CMD["PLUS_YAW"]))
        #self.min_pitch.clicked[bool].connect(lambda: self.move(CMD["MIN_PITCH"]))
        #self.plus_pitch.clicked[bool].connect(lambda: self.move(CMD["PLUS_PITCH"]))
        #self.min_roll.clicked[bool].connect(lambda: self.move(CMD["MIN_ROLL"]))
        #self.plus_roll.clicked[bool].connect(lambda: self.move(CMD["PLUS_ROLL"]))

        self.yaw_slider.valueChanged[int].connect(lambda: self.slider_callback())
        
        self.val_yaw.setText(str(0))
        self.val_pitch.setText(str(0)) 
        self.val_roll.setText(str(0))

        self.cmd_pub = rospy.Publisher("cmd_manual", Int32, queue_size=10)
        self.motor_sub = rospy.Subscriber("motors_position", Int32MultiArray, self.update_motors)

    def update_motors(self, state):
        # TODO : doesnt work
        self.val_yaw.setText(str(state.data[0]))
        self.val_pitch.setText(str(state.data[1])) 
        self.val_roll.setText(str(state.data[2]))
        pass

    def change_mode(self, mode):
        if mode == MANUAL:
            self.auto_btn.setEnabled(True)
            self.mode = MANUAL
            
        if mode == AUTO:
            self.auto_btn.setEnabled(False)
            self.mode = AUTO

    def enable_motors(self, enable):
        if self.disable_btn.text() == "Enable":
            self.cmd_pub.publish(CMD["ENABLE"])
            self.disable_btn.setText("Disable")

        elif self.disable_btn.text() == "Disable":
            self.cmd_pub.publish(CMD["DISABLE"])
            self.disable_btn.setText("Enable")

    def track_face(self):
        print("TRACK FACE")
        pass

    def move(self, dir):
        self.cmd_pub.publish(dir)

    # Call motor service
    def call_motor_cmd(self, id, command, value):
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command', 0.1)
        try:
            move_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            request = DynamixelCommandRequest()
            request.id = id
            request.addr_name = command
            request.value = value
            print request
            response = move_motor(request)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Enable motors
    def enable_motors(self, enable):
        self.call_motor_cmd(1, "Torque_Enable", enable)
        self.call_motor_cmd(2, "Torque_Enable", enable)
        self.call_motor_cmd(3, "Torque_Enable", enable)


    def slider_callback(self, axis, value):
        self.enable(True)
        self.call_motor_cmd(axis, "Goal_Position", value)

