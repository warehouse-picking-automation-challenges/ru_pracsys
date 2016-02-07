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

class MotomanAction(object):
  # create messages that are used to publish feedback/result
  _feedback = FollowJointTrajectoryFeedback()
  _result   = FollowJointTrajectoryResult()

  def __init__(self, name):
    self._action_name = name
    self._names = rospy.get_param('controller_joint_names')
    self._state = JointState(name=self._names, position=[0]*len(self._names))
    # print self._state.name
    #self._state.position = [1.57, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0, 1.57, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0]
    # self._state.position = [ 1.4936031103134155273437500,0.1488302350044250488281250,-2.1573691368103027343750000,-0.6662488579750061035156250,
    #                                                     -2.0533471107482910156250000,-0.9614110589027404785156250,-2.7441112995147705078125000,
    #                         1.4999946355819702148437500,1.2999976873397827148437500,0.0000151879285112954676151,-0.0000151879285112954676151,
    #                                                       0.0000000000000000000000000,0.0000000000000000000000000,0.0000000000000000000000000,
    #                         0.0446388423442840576171875,0.0446388423442840576171875]
    # print self._state.position
    self._joint_limits = rospy.get_param('robot_limits')
    # print self._joint_limits
    self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
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
          rospy.logerr('%s joint position is outside limits. %f not in [%f, %f]',trajectory.joint_names[x], target_point.positions[x], minimum, maximum)
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
          rospy.logerr('Motoman sim:  %s requested velocity %f (given by the time_from_start value) is too large (max %f). Aborting...',trajectory.joint_names[x],(target_point.positions[x] - current_point.positions[x])/(diff.secs+diff.nsecs/1000000000.0),vel)
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
        r.sleep()
      current_point = target_point
    ## start executing the action
    # for i in Trajectory
      ## check that preempt has not been requested by the client
      # if self._as.is_preempt_requested():
      #   rospy.loginfo('%s: Preempted' % self._action_name)
      #   self._as.set_preempted()
      #   success = False
      #   break
      # self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
      ## publish the feedback
      # self._as.publish_feedback(self._feedback)
      ## wait to set the new state.
      # r.sleep()

    #add new updated data into the current state

    if success:
      self._as.set_succeeded(self._result)
      rospy.loginfo("SUCCESS!!!")
    else:
      rospy.loginfo("FAILURE :( :( :(");
      
if __name__ == '__main__':
  rospy.init_node('joint_path_command')
  action = MotomanAction(rospy.get_name())
  pub = rospy.Publisher('/joint_states', JointState, queue_size=2)
  rate = rospy.Rate(50) # 10hz
  # robotiq_state = JointState(name=['finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3', 'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3', 'finger_middle_joint_1', 'finger_middle_joint_2', 'finger_middle_joint_3', 'palm_finger_1_joint', 'palm_finger_2_joint'], position=[0]*11)
  ##publish the state from the MotomanAction 
  while not rospy.is_shutdown():
    action._state.header.stamp = rospy.Time.now()
    pub.publish(action._state)
    # robotiq_state.header.stamp = rospy.Time.now()
    # pub.publish(robotiq_state)
    rate.sleep()
