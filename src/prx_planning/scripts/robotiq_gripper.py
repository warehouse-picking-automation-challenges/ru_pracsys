#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('robotiq_s_model_control')
from robotiq_s_model_control.msg import *

class RobotiqGripper():
    def __init__(self):
        print "Initializing the Robotiq Gripper"
        self.g_robotiq_state = SModel_robot_input();
        self.g_robotiq_state.gSTA = 3
        self.GM_basic = 0
        self.GM_pinch = 1
        self.GM_scissor = 2
        self.GM_wide = 3

        self.grasping_modes = [self.GM_basic, self.GM_pinch, self.GM_scissor, self.GM_wide]
        self.publisher = rospy.Publisher('/SModelRobotOutput', SModel_robot_output)
        self.subscriber = rospy.Subscriber('/SModelRobotInput', SModel_robot_input, self.robotiq_state_callback)
        self._current_grasping_mode = self.GM_basic
        rospy.sleep(2)

        #command = SModel_robot_output();
        #command.rACT = 1
        #command.rGTO = 1
        #command.rSPA = 250
        #command.rFRA = 150
        #self.publisher.publish(command)
        #rospy.sleep(1);
        #while True:
        #    state = self.g_robotiq_state
        #    if state.gSTA!=0:
        #        break
        self.fake_flag = False

    def robotiq_state_callback(self,robot_input):
        self.g_robotiq_state = robot_input 
    
    def open(self,open_type=""):
        if self.fake_flag:
            return
        command = SModel_robot_output()
        command.rACT = 1
        command.rGTO = 1
        if open_type == "bin_open":
            command.rPRA = 30
        if open_type == "order_open":
            command.rPRA = 0
        self.publisher.publish(command)
        rospy.sleep(1);
        while True:
            state = self.g_robotiq_state
            if state.gSTA!=0:
                break

    def close(self):
        if self.fake_flag:
            return
        command = SModel_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rPRA = 255
        command.rFRA = 0
        self.publisher.publish(command)
        rospy.sleep(1);
        while True:
            state = self.g_robotiq_state
            if state.gSTA!=0:
                break

    def reset(self):
        if self.fake_flag:
            return
        command = outputMsg.SModel_robot_output();
        command.rACT = 0
        
    def calibrate(self):
        return
