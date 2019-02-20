#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool, Int32, Int32MultiArray
from sensor_msgs.msg import Joy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from constants import *
from timeit import default_timer as timer


class CmdMux():
    def __init__(self):
        
        self.last_call = timer()
        self.delay = 0.03
        
        self.mode = MANUAL
        
        # Set max speed
        self.build_motor_cmd(1, "Position_P_Gain", 300)
        self.build_motor_cmd(2, "Position_P_Gain", 350)
        self.build_motor_cmd(3, "Position_P_Gain", 400)
        
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        
        self.motor_sub = rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.update_motors)
        self.man_sub = rospy.Subscriber("cmd_manual", DynamixelCommandRequest, self.ui_cmd)
        self.mode_auto = rospy.Subscriber("auto_mode", Bool, self.auto_mode)
        self.motor_pub = rospy.Publisher("motors_position", Int32MultiArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_motor)

        rospy.spin()  
        
    # Read motor state    
    def update_motors(self, state):
        for motor in state.dynamixel_state:
            if motor.name == "yaw":
                self.yaw = motor.present_position 
            
            elif motor.name == "pitch":
                self.pitch = motor.present_position 
            
            elif motor.name == "roll":
                self.roll = motor.present_position 
                
    # Publish position for UI            
    def publish_motor(self, event):
        msg = Int32MultiArray()
        msg.data = [((self.yaw-YAW_HOME)/STEP), ((self.pitch-PITCH_HOME)/STEP), ((self.roll-ROLL_HOME)/STEP)]
        self.motor_pub.publish(msg)
        
     # Move with the UI buttons        
    def ui_cmd(self, request):
        self.mode = MANUAL
        self.call_motor_cmd(request)
        
    # Enable motors
    def enable_motors(self, enable):
        self.call_motor_cmd(1, "Torque_Enable", enable)
        self.call_motor_cmd(2, "Torque_Enable", enable)
        self.call_motor_cmd(3, "Torque_Enable", enable)
        
    def build_motor_cmd(self, id, cmd, val):
        request = DynamixelCommandRequest()
        request.id = id
        request.addr_name = cmd
        request.value = val
        self.call_motor_cmd(request)
 
    # Call motor service
    def call_motor_cmd(self, request):
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command', 0.1)
        try:
            move_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            response = move_motor(request)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def auto_mode(self, auto):
        if auto:
            self.mode = AUTO
        else:
            self.mode = MANUAL
        

if __name__ == '__main__':
    # Initialize the node and name it.

    print("Cmd mux started")
    rospy.init_node('cmd_mux')

    try:
        mux = CmdMux()
    except rospy.ROSInterruptException:
        pass
