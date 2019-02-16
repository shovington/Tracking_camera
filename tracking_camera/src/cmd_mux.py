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
        self.motor_sub = rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.update_motors)
        self.man_sub = rospy.Subscriber("cmd_manual", Int32, self.move_man)
        self.controller_sub = rospy.Subscriber("/joy", Joy, self.move_controller )
        
        self.motor_pub = rospy.Publisher("motors_position", Int32MultiArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_motor)
        
        # Set max speed
        self.call_motor_cmd(1, "Position_P_Gain", 160)
        self.call_motor_cmd(2, "Position_P_Gain", 250)
        self.call_motor_cmd(3, "Position_P_Gain", 400)
        
        self.yaw = 0
        self.pitch = 0
        self.roll = 0

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
        
    # Move gimbal with the xbox controller    
    def move_controller(self, joy):
        if timer() - self.last_call > self.delay:
            if joy.buttons[XBOX["HOME"]]:
                self.enable_motors(1)
                self.call_motor_cmd(1, "Goal_Position", YAW_HOME)
                self.call_motor_cmd(2, "Goal_Position", PITCH_HOME)
                self.call_motor_cmd(3, "Goal_Position", ROLL_HOME)
                
            elif joy.buttons[XBOX["A"]]:
                self.enable_motors(1)
                self.call_motor_cmd(2, "Goal_Position", self.pitch - 15*STEP)
                
            elif joy.buttons[XBOX["B"]]:
                self.enable_motors(1)
                self.call_motor_cmd(3, "Goal_Position", self.roll + 15*STEP)
                
            elif joy.buttons[XBOX["X"]]:
                self.enable_motors(1)
                self.call_motor_cmd(3, "Goal_Position",self.roll - 15*STEP)
                
            elif joy.buttons[XBOX["Y"]]:
                self.enable_motors(1)
                self.call_motor_cmd(2, "Goal_Position", self.pitch + 15*STEP)
                
            elif joy.buttons[XBOX["LB"]]:
                self.enable_motors(1)
                self.call_motor_cmd(1, "Goal_Position", self.yaw - 15*STEP)
                
            elif joy.buttons[XBOX["RB"]]:
                self.enable_motors(1)
                self.call_motor_cmd(1, "Goal_Position", self.yaw + 15*STEP)
                
            self.last_call = timer()
            
    # Move with the UI buttons        
    def move_man(self, dir):
        if timer() - self.last_call > self.delay:   
            self.enable_motors(1)
            if dir.data == CMD["HOME"]:
                self.call_motor_cmd(1, "Goal_Position", YAW_HOME)
                rospy.sleep(0.5)
                self.call_motor_cmd(2, "Goal_Position", PITCH_HOME)
                rospy.sleep(0.5)
                self.call_motor_cmd(3, "Goal_Position", ROLL_HOME)
    
            elif dir.data == CMD["MIN_YAW"]:
                self.call_motor_cmd(1, "Goal_Position", self.yaw - 15*STEP)
    
            elif dir.data == CMD["PLUS_YAW"]:
                self.call_motor_cmd(1, "Goal_Position", self.yaw + 15*STEP)
    
            elif dir.data == CMD["MIN_PITCH"]:
                self.call_motor_cmd(2, "Goal_Position", self.pitch - 15*STEP)
    
            elif dir.data == CMD["PLUS_PITCH"]:
                self.call_motor_cmd(2, "Goal_Position", self.pitch + 15*STEP)
    
            elif dir.data == CMD["MIN_ROLL"]:
                self.call_motor_cmd(3, "Goal_Position",self.roll - 15*STEP)
    
            elif dir.data == CMD["PLUS_ROLL"]:
                self.call_motor_cmd(3, "Goal_Position", self.roll + 15*STEP)
              
            elif dir.data == CMD["ENABLE"]:
              self.enable_motors(1)
    
            elif dir.data == CMD["DISABLE"]:
              self.enable_motors(0)
              
            self.last_call = timer()

    # Enable motors
    def enable_motors(self, enable):
        self.call_motor_cmd(1, "Torque_Enable", enable)
        self.call_motor_cmd(2, "Torque_Enable", enable)
        self.call_motor_cmd(3, "Torque_Enable", enable)
 
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
        

if __name__ == '__main__':
    # Initialize the node and name it.

    print("Cmd mux started")
    rospy.init_node('cmd_mux')

    try:
        mux = CmdMux()
    except rospy.ROSInterruptException:
        pass
