#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool, Int32, Int32MultiArray
from sensor_msgs.msg import Joy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from constants import *
from timeit import default_timer as timer


class MotorControl():
    def __init__(self):
        self.motors = 0

        try:
            rospy.wait_for_service('/dynamixel_workbench/dynamixel_command', 0.1)
            self.motor_proxy = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            self.motors = 1
        except:
            self.motors = 0
            print("WAITING FOR DYNAMIXEL MOTORS")
            self.timer = rospy.Timer(rospy.Duration(2), self.connect)

        # Set max speed
        self.call_motor_cmd(1, "Position_P_Gain", 300)
        self.call_motor_cmd(2, "Position_P_Gain", 350)
        self.call_motor_cmd(3, "Position_P_Gain", 400)
        
        self.angles = [0, 0, 0]
        self.limits = [YAW, PITCH, ROLL]

    def connect(self, event):
        try:
            rospy.wait_for_service('/dynamixel_workbench/dynamixel_command', 0.1)
            self.motor_proxy = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            self.timer.shutdown()
            self.motors = 1
            print("MOTORS FOUND")
        except:
            self.motors = 0
            print("WAITING FOR DYNAMIXEL MOTORS")

    def update_motors(self, angles):
        self.angles = angles

    # Enable motors
    def enable_motors(self, enable):
        
        request = [0, 0, 0]
        request[0] = DynamixelCommandRequest()
        request[1] = DynamixelCommandRequest()
        request[2] = DynamixelCommandRequest()
        
        request[0] = self.call_motor_cmd(1, "Torque_Enable", enable)
        request[1] = self.call_motor_cmd(2, "Torque_Enable", enable)
        request[2] = self.call_motor_cmd(3, "Torque_Enable", enable)
        return request

    def go_home(self):
        
        request = [0, 0, 0]
        request[0] = DynamixelCommandRequest()
        request[1] = DynamixelCommandRequest()
        request[2] = DynamixelCommandRequest()

        request[0] = self.call_motor_cmd(1, "Goal_Position", YAW_HOME)  
        request[1] = self.call_motor_cmd(2, "Goal_Position", PITCH_HOME)  
        request[2] = self.call_motor_cmd(3, "Goal_Position", ROLL_HOME)
        return request

    def move_axis(self, axis, value, limits):

        request = DynamixelCommandRequest()
        request = self.call_motor_cmd(axis, "Goal_Position", value*2048/SLIDER_RANGE + HOME[axis-1])
        return request
        
    def move_relative(self, axis, value):
        
        request = DynamixelCommandRequest()

        if self.limits[axis-1][1] < value+ self.angles[axis-1] or value+ self.angles[axis-1] < self.limits[axis-1][0]:
            print("Out of range")
            request.id = 0
            request.addr_name = 0
            request.value = 0
            return request
        else:
            request = self.call_motor_cmd(axis, "Goal_Position", value + self.angles[axis-1])
            return request

    # Call motor service
    def call_motor_cmd(self, id, cmd, val):
        if self.motors == 0:
            request = DynamixelCommandRequest()
            request.id = id
            request.addr_name = cmd
            request.value = val
            return request

        try:           
            response = self.motor_proxy(request)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            

