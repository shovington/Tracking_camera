import os
import rospkg
import rospy
import rosnode
from rospkg import RosPack
from qt_gui.plugin import Plugin
import subprocess
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets

from .tracking_cam import TrackingCamWidget
from .motor_control import MotorControl
from .track_face import TrackFace
from constants import *
from dynamixel_workbench_msgs.msg import *
from dynamixel_workbench_msgs.srv import *
from darknet_ros_msgs.msg import BoundingBoxes


class TrackingCamPlugin(Plugin):
    """
        Application window controller
    """

    def __init__(self, context):
        super(TrackingCamPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TrackingCamPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create Application main
        self.main = TrackingCamMain()
        self._widget = self.main.get_view()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    ####### Tutorial functions #########
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        # Stopping the path_creator node
        rospy.signal_shutdown('Shutting down')

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog


class TrackingCamMain():
    def __init__(self):
        self.view = TrackingCamWidget(self)
        self.motor_control = MotorControl()
	self.face_tracker = TrackFace()

        self.mode = MANUAL

        self.motor_sub = rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.update_motors)
        self.box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.box_callback)

    def get_view(self):
        return self.view

    def change_mode(self, mode):
    	self.mode = mode

    # Read motor state    
    def update_motors(self, state):
        for motor in state.dynamixel_state:
            if motor.name == "yaw":
                yaw = motor.present_position 
            elif motor.name == "pitch":
                pitch = motor.present_position 
            elif motor.name == "roll":
                roll = motor.present_position 

        self.view.update_motors([yaw, pitch, roll])
        self.motor_control.update_motors([yaw, pitch, roll])

    def go_home(self):
        self.change_mode(MANUAL)
        self.motor_control.go_home()
     
    # Enable motors
    def enable_motors(self, enable):
        self.motor_control.enable_motors(enable)

    def box_callback(self, box):
	if self.mode == AUTO:
	    offset = self.face_tracker.box_offset(box)
            if abs(offset[0]) > 20:
                self.motor_control.move_relative(1, -0.5*offset[0])
            if abs(offset[1]) > 20:
                self.motor_control.move_relative(2, -0.5*offset[1])

    def move_axis(self, axis, value, limits):
        self.change_mode(MANUAL)
        self.motor_control.move_axis(axis, value, limits)




