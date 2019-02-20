import os
import rospkg
import rospy
from rospkg import RosPack

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtCore import QCoreApplication
from PyQt5 import QtCore, QtGui, QtWidgets

from std_msgs.msg import Float32, Bool, Int32, String
from constants import * 
from dynamixel_workbench_msgs.msg import *
from dynamixel_workbench_msgs.srv import *
from std_msgs.msg import Float32, Bool, Int32, Int32MultiArray


class TrackingCamWidget(QtWidgets.QWidget):
    def __init__(self):
        super(TrackingCamWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('tracking_cam_gui'), 'resource', 'tracking_cam.ui')
        loadUi(ui_file, self)

        rp = RosPack()
        self.pkg_path = rp.get_path('tracking_cam_gui')

        self.mode = MANUAL  # Manual mode

        self.disable_btn.clicked[bool].connect(self.enable_motors_callback)
        self.auto_btn.clicked[bool].connect(lambda: self.change_mode(AUTO))
        self.track_btn.clicked[bool].connect(self.track_face)
        self.home_btn.clicked[bool].connect(self.go_home)

        self.yaw_slider.valueChanged[int].connect(self.slider_callback_yaw)
        self.pitch_slider.valueChanged[int].connect(self.slider_callback_pitch)
        self.roll_slider.valueChanged[int].connect(self.slider_callback_roll)
        
        self.val_yaw.setText(str(0))
        self.val_pitch.setText(str(0)) 
        self.val_roll.setText(str(0))

        self.cmd_pub = rospy.Publisher("cmd_manual", DynamixelCommandRequest, queue_size=10)
        self.auto_pub = rospy.Publisher("auto_mode", Bool, queue_size=10)
        self.motor_sub = rospy.Subscriber("motors_position", Int32MultiArray, self.update_motors)

    def update_motors(self, state):
        # TODO : doesnt work
        self.val_yaw.setText(str(state.data[0]))
        self.val_pitch.setText(str(state.data[1])) 
        self.val_roll.setText(str(state.data[2]))
        
    def go_home(self):
        self.change_mode(MANUAL)
        self.call_motor_cmd(1, "Goal_Position", YAW_HOME)  
        self.call_motor_cmd(2, "Goal_Position", PITCH_HOME)  
        self.call_motor_cmd(3, "Goal_Position", ROLL_HOME) 
        self.yaw_slider.setValue(50)
        self.pitch_slider.setValue(50)
        self.roll_slider.setValue(50)

    def change_mode(self, mode):
        if mode == MANUAL:
            self.auto_btn.setEnabled(True)
            self.mode = MANUAL
            
        if mode == AUTO:
            self.auto_btn.setEnabled(False)
            self.mode = AUTO
            self.auto_pub.publish(True)

    def enable_motors_callback(self, enable):
        if self.disable_btn.text() == "Enable":
            self.enable_motors(True)
            self.disable_btn.setText("Disable")

        elif self.disable_btn.text() == "Disable":
            self.enable_motors(False)
            self.disable_btn.setText("Enable")

    def track_face(self):
        #TODO
        print("TRACK FACE")
        pass

    # Call motor service
    def call_motor_cmd(self, id, command, value):
        request = DynamixelCommandRequest()
        request.id = id
        request.addr_name = command
        request.value = value
        self.cmd_pub.publish(request)
        
    def direct_cmd(self, id, command, value):
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command', 0.1)
        try:
            move_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            request = DynamixelCommandRequest()
            request.id = id
            request.addr_name = command
            request.value = value
            self.cmd_pub.publish(request)
            response = move_motor(request)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Enable motors
    def enable_motors(self, enable):
        self.call_motor_cmd(1, "Torque_Enable", enable)
        self.call_motor_cmd(2, "Torque_Enable", enable)
        self.call_motor_cmd(3, "Torque_Enable", enable)

    def slider_callback_yaw(self, value):
        self.change_mode(MANUAL)
        self.call_motor_cmd(1, "Goal_Position", (value-50)*2048/100 + YAW_HOME)    

    def slider_callback_pitch(self, value):
        self.change_mode(MANUAL)
        self.call_motor_cmd(2, "Goal_Position", (value-50)*2048/100 + PITCH_HOME) 

    def slider_callback_roll(self, value):
        self.change_mode(MANUAL)
        self.call_motor_cmd(3, "Goal_Position", (value-50)*2048/100 + ROLL_HOME) 

