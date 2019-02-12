#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool, Int32
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandRequest
from constants import *


class CmdMux():
    def __init__(self):

        self.mode = MANUAL
        self.man_sub = rospy.Subscriber("cmd_manual", Int32, self.move_man)
        # self.motors_sub = rospy.Subscriber("TODO", Type, self.update_info) #TODO

        rospy.spin()
     
        # Set max speed
        self.call_motor_cmd(1, "Velocity_Limit", MAX_SPEED)
        rospy.sleep(0.5)
        self.call_motor_cmd(2, "Velocity_Limit", MAX_SPEED)
        rospy.sleep(0.5)
        self.call_motor_cmd(3, "Velocity_Limit", MAX_SPEED)

    def move_man(self, dir):
        if dir.data == CMD["HOME"]:
            self.enable_motors(1)
            self.call_motor_cmd(1, "Goal_Position", YAW_HOME)
            rospy.sleep(0.5)
            self.call_motor_cmd(2, "Goal_Position", PITCH_HOME)
            rospy.sleep(0.5)
            self.call_motor_cmd(3, "Goal_Position", ROLL_HOME)

        elif dir.data == CMD["MIN_YAW"]:
            self.call_motor_cmd(1, "Goal_Position", YAW_HOME)

        elif dir.data == CMD["PLUS_YAW"]:
            self.call_motor_cmd(1, "Goal_Position", YAW_HOME)

        elif dir.data == CMD["MIN_PITCH"]:
            self.call_motor_cmd(2, "Goal_Position", PITCH_HOME)

        elif dir.data == CMD["PLUS_PITCH"]:
            self.call_motor_cmd(2, "Goal_Position", PITCH_HOME)

        elif dir.data == CMD["MIN_ROLL"]:
            self.call_motor_cmd(3, "Goal_Position", ROLL_HOME)

        elif dir.data == CMD["PLUS_ROLL"]:
            self.call_motor_cmd(3, "Goal_Position", ROLL_HOME)
          
        elif dir.data == CMD["ENABLE"]:
          self.enable_motors(1)

        elif dir.data == CMD["DISABLE"]:
          self.enable_motors(0)

    def enable_motors(self, enable):
        self.call_motor_cmd(1, "Torque_Enable", enable)
        self.call_motor_cmd(2, "Torque_Enable", enable)
        self.call_motor_cmd(3, "Torque_Enable", enable)
 
    def call_motor_cmd(self, id, command, value):
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
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
      

if __name__ == '__main__':
    # Initialize the node and name it.

    print("Cmd mux started")
    rospy.init_node('cmd_mux')

    try:
        mux = CmdMux()
    except rospy.ROSInterruptException:
        pass
