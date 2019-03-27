#!/usr/bin/env python

import sys
import unittest
from tracking_cam import TrackingCamWidget
from motor_control import MotorControl
from constants import *
#from track_face import TrackFace
#from tracking_cam_pluging import TrackingCamMain


## A sample python unit test
class TestTrackingCam(unittest.TestCase):

    def setUp(self):
        #self.main = TrackingCamMain()
        #self.view = TrackingCamWidget()
        self.motor_control = MotorControl()
	    #self.face_tracker = TrackFace()
    
    def test_motor_control_enable_motors(self):
        
        request = self.motor_control.enable_motors(True)
        self.assertEquals(request[0].id, 1, "Wrong id")
        self.assertEquals(request[1].id, 2, "Wrong id")
        self.assertEquals(request[2].id, 3, "Wrong id")
        self.assertEquals(request[0].addr_name, "Torque_Enable", "Wrong service")
        self.assertEquals(request[0].value, True, "Wrong value")
        request = self.motor_control.enable_motors(False)
        self.assertEquals(request[0].value, False, "Wrong value")
        
    def test_motor_control_go_home(self):
        
        request = self.motor_control.go_home()
        self.assertEquals(request[0].id, 1, "Wrong id")
        self.assertEquals(request[1].id, 2, "Wrong id")
        self.assertEquals(request[2].id, 3, "Wrong id")
        self.assertEquals(request[0].addr_name, "Goal_Position", "Wrong service")
        self.assertEquals(request[0].value, YAW_HOME, "Wrong value")
        self.assertEquals(request[1].value, PITCH_HOME, "Wrong value")
        self.assertEquals(request[2].value, ROLL_HOME, "Wrong value")
        
    def test_motor_control_move_axis(self):
        
        axis = 1
        value = 10
        limits = [YAW, PITCH, ROLL]
        wanted_value = value*2048/SLIDER_RANGE + HOME[axis-1]
        
        request = self.motor_control.move_axis(axis, value, limits)
        self.assertEquals(request.id, axis, "Wrong id")
        self.assertEquals(request.addr_name, "Goal_Position", "Wrong service")
        self.assertEquals(request.value, wanted_value, "Wrong value")
    
    def test_motor_control_move_relative_OUTOFRANGE(self):
        
        axis = 1
        value = YAW_HOME+RANGE
        
        request = self.motor_control.move_relative(axis, value)
        self.assertEquals(request.id, 0, "Wrong id - it should be 0")
        self.assertEquals(request.addr_name, 0, "Wrong service - it should be 0")
        self.assertEquals(request.value, 0, "Wrong value - it should be 0")
        
    def test_motor_control_move_relative_INRANGE(self):
        axis = 1
        value = YAW_HOME
        
        request = self.motor_control.move_relative(axis, value)
        self.assertEquals(request.id, 1, "Wrong id")
        self.assertEquals(request.addr_name, "Goal_Position", "Wrong service")
        self.assertEquals(request.value, YAW_HOME, "Wrong value")
        


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("trackin_camera", 'TestTrackingCam', TestTrackingCam)
