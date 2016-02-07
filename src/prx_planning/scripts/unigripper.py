#!/usr/bin/env python

import rospy

from unigripper_msgs.srv import *

class UniGripper():
    def __init__(self):
        print "Initializing the UniGripper"
    
    def turn_off(self,open_type=""):
        rospy.wait_for_service('unigripper_vacuum')
        try:
            open_unigripper = rospy.ServiceProxy('unigripper_vacuum', UnigripperVacuumOn)
            resp = open_unigripper(False)
            return resp.VacuumState
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def turn_on(self):
        rospy.wait_for_service('unigripper_vacuum')
        try:
            open_unigripper = rospy.ServiceProxy('unigripper_vacuum', UnigripperVacuumOn)
            resp = open_unigripper(True)
            return resp.VacuumState
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def reset(self):
        rospy.wait_for_service('unigripper_vacuum')
        try:
            open_unigripper = rospy.ServiceProxy('unigripper_vacuum', UnigripperVacuumOn)
            resp = open_unigripper(False)
            return resp.VacuumState
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def calibrate(self):
        pass