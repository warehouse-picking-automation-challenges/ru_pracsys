#! /usr/bin/env python

import roslib
import rospy
import math

from sensor_msgs.msg import JointState

if __name__ == '__main__':
  rospy.init_node('robotiq_updater')
  pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
  rate = rospy.Rate(10) # 10hz
  robotiq_state = JointState(name=['finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3', 'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3', 'finger_middle_joint_1', 'finger_middle_joint_2', 'finger_middle_joint_3', 'palm_finger_1_joint', 'palm_finger_2_joint'], position=[0]*11)
  while not rospy.is_shutdown():
    robotiq_state.header.stamp = rospy.Time.now()
    pub.publish(robotiq_state)
    rate.sleep()
