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
        
        self.last_call = timer()
        self.delay = 0.03
        
        # Set max speed
        self.call_motor_cmd(1, "Position_P_Gain", 300)
        self.call_motor_cmd(2, "Position_P_Gain", 350)
        self.call_motor_cmd(3, "Position_P_Gain", 400)
        
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        
    def update_motors(self, angles):
        self.yaw = angles[0]
        self.pitch = angles[1]
        self.roll = angles[2]

    # Enable motors
    def enable_motors(self, enable):
        self.call_motor_cmd(1, "Torque_Enable", enable)
        self.call_motor_cmd(2, "Torque_Enable", enable)
        self.call_motor_cmd(3, "Torque_Enable", enable)

    def go_home(self):
        self.call_motor_cmd(1, "Goal_Position", YAW_HOME)  
        self.call_motor_cmd(2, "Goal_Position", PITCH_HOME)  
        self.call_motor_cmd(3, "Goal_Position", ROLL_HOME)

    def move_axis(self, axis, value, limits):
        self.call_motor_cmd(axis, "Goal_Position", value*2048/(limits[1]-limits[0]) + HOME[axis-1])
        
    # Call motor service
    def call_motor_cmd(self, id, cmd, val):
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command', 0.1)
        try:
            move_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            request = DynamixelCommandRequest()
            request.id = id
            request.addr_name = cmd
            request.value = val
            response = move_motor(request)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            

