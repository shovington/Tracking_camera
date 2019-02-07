#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool, Int32
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandRequest


MANUAL = 0
AUTO = 1

CMD = {"MIN_YAW": 0,
       "PLUS_YAW": 1,
       "MIN_PITCH": 2,
       "PLUS_PITCH": 3,
       "MIN_ROLL": 4,
       "PLUS_ROLL": 5,
       "HOME":6,
       "ENABLE":7}

MAX_SPEED = 30


class CmdMux():
    def __init__(self):
        self.mode = MANUAL

        self.man_sub = rospy.Subscriber("cmd_manual", Int32, self.move_man)

        self.cmd_publisher = rospy.Publisher("test", Int32, queue_size=10)
     
        # Set max speed
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        try:
            move_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            request1 = DynamixelCommandRequest()
            request1.id = 1
            request1.addr_name = "Velocity_Limit"
            request1.value = MAX_SPEED
            print request1
            response = move_motor(request1)
            rospy.sleep(0.5)

            request2 = DynamixelCommandRequest()
            request2.id = 2
            request2.addr_name = "Velocity_Limit"
            request2.value = MAX_SPEED
            print request2
            response = move_motor(request2)
            rospy.sleep(0.5)

            request3 = DynamixelCommandRequest()
            request3.id = 3
            request3.addr_name = "Velocity_Limit"
            request3.value = MAX_SPEED
            print request3
            response = move_motor(request3)
          
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.spin()

    def move_man(self, dir):
        if dir.data == CMD["HOME"]:
            self.enable_motors(1)
            self.call_motor_cmd(1, "Goal_Position", 2048)
            rospy.sleep(0.5)
            self.call_motor_cmd(2, "Goal_Position", 2800)
            rospy.sleep(0.5)
            self.call_motor_cmd(3, "Goal_Position", 3072)

        elif dir.data == CMD["ENABLE"]:
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
