#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool, Int32


MANUAL = 0
AUTO = 1

CMD = {"MIN_YAW": 0,
       "PLUS_YAW": 1,
       "MIN_PITCH": 2,
       "PLUS_PITCH": 3,
       "MIN_ROLL": 4,
       "PLUS_ROLL": 5,
       "HOME":6}


class CmdMux():
    def __init__(self):
        self.mode = MANUAL

        self.man_sub = rospy.Subscriber("cmd_manual", Int32, self.move_man)

        self.cmd_publisher = rospy.Publisher("test", Int32)
        rospy.spin()

      def man_mouv(self, dir):
        if dir == CMD[HOME]:
          rospy.wait_for_service('dynamixel_command')
          try:
              move_yaw = rospy.ServiceProxy('dynamixel_command', DynamixelCommand)
              request = DynamixelCommandRequest()
              request.id = 1
              request.addr_name = "Goal_Position"
              request.value = 0
              print request
              response = call_load_file(request)
          
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