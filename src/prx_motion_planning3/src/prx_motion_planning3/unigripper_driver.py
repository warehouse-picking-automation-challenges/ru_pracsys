#! /usr/bin/env python

import roslib
import rospy
import math

import actionlib

# import actionlib_tutorials.msg
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from motoman_msgs.srv import CmdJointTrajectoryEx
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult
)

from servo_device import Controller
from unigripper_msgs.srv import *


class MotomanAction(object):
  # create messages that are used to publish feedback/result
  _feedback = FollowJointTrajectoryFeedback()
  _result   = FollowJointTrajectoryResult()

  def __init__(self, name):
    self._action_name = name
    self._names = ['head_hinge']
    self._state = JointState(name=self._names, position=[0]*len(self._names))
    # print self._state.position
    self._joint_limits = {'head_hinge':{'min':0,'max':1.57,'vel':.4}}
    # print self._joint_limits
    self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

    self.servo = Controller()
    self.servo.setAccel(1, 0) # hinge servo
    self.servo.setSpeed(1, 0) # hinge servo
    self.set_servo_state(self._state.position[0]) # head_hinge
    print "@@@@@@@@@@@@@ Initial head_hinge position = ", self._state.position[0]


  def set_servo_state(self, angle):
    # zero angle in servo values = 4000; 90 degrees angle in servo value = 7600
    # value in between are just an approximate...
    one_servo_radian = (7600 - 4000) / (math.pi / 2.0)
    servo_value = int(4000 + angle * one_servo_radian)
    # print "@@@@@@@@@@@@@ servo_value = ", servo_value, "   angle = ", angle
    self.servo.setTarget(1, servo_value)


  def handle_unigripper_vacuum(self, req):
    if req.TurnVacuumOn == True:
      self.servo.setTarget(0, 7600)
      print "Turned vacuum on"
      return UnigripperVacuumOnResponse(True)
    else:
      self.servo.setTarget(0, 4000)
      print "Turned vacuum off"
      return UnigripperVacuumOnResponse(False)
    

  def execute_cb(self, goal):
    rospy.loginfo('Received Trajectory')
    ## helper variables
    r = rospy.Rate(100)
    success = True
        
    ## simulate the execution of the trajectory in goal
    trajectory = goal.trajectory
    current_point = trajectory.points[0]

    for x in xrange(len(trajectory.joint_names)):
      if abs(self._state.position[self._state.name.index(trajectory.joint_names[x])] - current_point.positions[x]) > .001:
        rospy.logerr('Simulator: Start State of Joint Trajectory does not correspond to true state of the robot. Aborting trajectory...')
        print "Current: " + str(self._state)
        print "Traj: " + str(current_point)
        print trajectory.joint_names
        success = False
        break

    for outer_loop in xrange(1,len(trajectory.points)):
      target_point = trajectory.points[outer_loop]
      for x in xrange(len(trajectory.joint_names)):
        minimum = self._joint_limits[trajectory.joint_names[x]]['min']
        maximum = self._joint_limits[trajectory.joint_names[x]]['max']
        if target_point.positions[x] < minimum or target_point.positions[x] > maximum:
          rospy.logerr('%s joint position is outside limits. Aborting...',trajectory.joint_names[x])
          success = False
          break
      diff = (target_point.time_from_start - current_point.time_from_start)
      for x in xrange(len(trajectory.joint_names)):
        vel = self._joint_limits[trajectory.joint_names[x]]['vel']
        if math.fabs(target_point.positions[x] - current_point.positions[x])/(diff.secs+diff.nsecs/1000000000.0) > vel:
          print (diff.secs+diff.nsecs/1000000000.0)
          print (target_point.positions[x] - current_point.positions[x])
          print target_point.positions[x]
          print current_point.positions[x]
          rospy.logerr('Unigripper driver:  %s requested velocity %f (given by the time_from_start value) is too large (max %f). Aborting...',trajectory.joint_names[x],(target_point.positions[x] - current_point.positions[x])/(diff.secs+diff.nsecs/1000000000.0),vel)
          success = False
          break

      if not success:
        break
      num_iters = int((diff.secs+diff.nsecs/1000000000.0)/.01)
      # print num_iters
      for iter_count in xrange(num_iters):
        if self._as.is_preempt_requested():
          self._as.set_preempted()
          rospy.loginfo('Preempted')
          success = False
          break
        for x in xrange(len(trajectory.joint_names)):
          val = (target_point.positions[x] - current_point.positions[x])/num_iters
          index = self._state.name.index(trajectory.joint_names[x])
          self._state.position[index] = self._state.position[index] + val

        self.set_servo_state(self._state.position[0])
        r.sleep()
      current_point = target_point

    if success:
      self._as.set_succeeded(self._result)
      rospy.loginfo("SUCCESS!!!")
    else:
      rospy.loginfo("FAILURE :( :( :(");
      
            
if __name__ == '__main__':
  rospy.init_node('unigripper_joint_path_command')
  action = MotomanAction("unigripper" + rospy.get_name())
  pub = rospy.Publisher('/joint_states', JointState, queue_size=2)
  s = rospy.Service('unigripper_vacuum', UnigripperVacuumOn, action.handle_unigripper_vacuum)
  rate = rospy.Rate(10) # 10hz
  # robotiq_state = JointState(name=['finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3', 'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3', 'finger_middle_joint_1', 'finger_middle_joint_2', 'finger_middle_joint_3', 'palm_finger_1_joint', 'palm_finger_2_joint'], position=[0]*11)
  ##publish the state from the MotomanAction 
  while not rospy.is_shutdown():
    action._state.header.stamp = rospy.Time.now()
    pub.publish(action._state)
    # robotiq_state.header.stamp = rospy.Time.now()
    # pub.publish(robotiq_state)
    rate.sleep()
