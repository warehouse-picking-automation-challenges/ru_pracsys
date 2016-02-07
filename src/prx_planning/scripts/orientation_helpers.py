#!/usr/bin/env python

import tf
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
import math
import copy
import rospy

def rotate_quaternion(q1,q2):
    q3 = copy.deepcopy(q1)
    q3.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y
    q3.y = q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z
    q3.z = q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x
    q3.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
    return q3

def axis_angle_to_quaternion(axis,angle):
    q = Quaternion()
    q.x = axis.x * math.sin(angle/2.0)
    q.y = axis.y * math.sin(angle/2.0)
    q.z = axis.z * math.sin(angle/2.0)
    q.w = math.cos(angle/2.0)
    return q

def get_tf_transform(from_frame, to_frame):
    tf_listener = tf.TransformListener()
    
    for i in range(0, 10):
        joy = True
        try:
            (trans,rot) = tf_listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            joy = False
        print "Trying tf lookup, transform '" + from_frame + "' -> '" + to_frame + "'"
        if joy == False and i == 9:
            print "Could not get tf transform after 9 tries..."
            return False
        elif joy == True:
            break
        rospy.sleep(0.5)

    transform = Pose()
    transform.position.x = trans[0]
    transform.position.y = trans[1]
    transform.position.z = trans[2]
    transform.orientation.x = rot[0]
    transform.orientation.y = rot[1]
    transform.orientation.z = rot[2]
    transform.orientation.w = rot[3]
    
    return transform

def rotate_orientation_quaternion(orientation):

    first_rotating_axis = 2
    second_rotating_axis = 2
    first_rotating_axis_signal = 1
    second_rotating_axis_signal = 1
    rotation1 = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    rotation2 = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    theta = math.pi / 2.0
    if first_rotating_axis == 1:
        rotation1 = tf.transformations.quaternion_from_euler(first_rotating_axis_signal * theta, 0.0, 0.0)
    elif first_rotating_axis == 2:
        rotation1 = tf.transformations.quaternion_from_euler(0.0, first_rotating_axis_signal * theta, 0.0)
    elif first_rotating_axis == 3:
        rotation1 = tf.transformations.quaternion_from_euler(0.0, 0.0, first_rotating_axis_signal * theta)

    if second_rotating_axis == 1:
        rotation2 = tf.transformations.quaternion_from_euler(second_rotating_axis_signal * theta, 0.0, 0.0)
    elif second_rotating_axis == 2:
        rotation2 = tf.transformations.quaternion_from_euler(0.0, second_rotating_axis_signal * theta, 0.0)
    elif second_rotating_axis == 3:
        rotation2 = tf.transformations.quaternion_from_euler(0.0, 0.0, second_rotating_axis_signal * theta)

    _orientation = copy.deepcopy(rotation1) # just to allocate this variable
    _orientation[0] = orientation.x
    _orientation[1] = orientation.y
    _orientation[2] = orientation.z
    _orientation[3] = orientation.w
    new_orientation = tf.transformations.quaternion_multiply(_orientation, rotation1)
    new_orientation = tf.transformations.quaternion_multiply(new_orientation, rotation2)
    orientation_rotated = Quaternion()
    orientation_rotated.x = new_orientation[0]
    orientation_rotated.y = new_orientation[1]
    orientation_rotated.z = new_orientation[2]
    orientation_rotated.w = new_orientation[3]

    return orientation_rotated

def position_as_list(position):
    return [position.x, position.y, position.z]

def quat_from_orient(orientation):
    return [orientation.x, orientation.y, orientation.z, orientation.w]
