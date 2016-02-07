 #!/usr/bin/env python

import sys 
import rospy

import math
import numpy
import time
import copy
#import baxter_interface
#from baxter_interface import CHECK_VERSION
import math
import tf
from prx_motion_planning3 import *

import std_msgs.msg
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

# from planning_library_interface import *
# from pracsys_library_interface import *

def rotate_orientation(orientation, rotation):
    _orientation = numpy.empty((4, ), dtype=numpy.float64)
    _orientation[0] = orientation.x
    _orientation[1] = orientation.y
    _orientation[2] = orientation.z
    _orientation[3] = orientation.w

    _rotation = numpy.empty((4, ), dtype=numpy.float64)
    _rotation[0] = rotation.x
    _rotation[1] = rotation.y
    _rotation[2] = rotation.z
    _rotation[3] = rotation.w

    new_orientation = tf.transformations.quaternion_multiply(_orientation, _rotation)

    orientation_rotated = Quaternion()
    orientation_rotated.x = new_orientation[0]
    orientation_rotated.y = new_orientation[1]
    orientation_rotated.z = new_orientation[2]
    orientation_rotated.w = new_orientation[3]

    return orientation_rotated

# def test_grasp(end_effector_pose, speed, ik_seed):
#     global g_move_group

#     local_end_effector_pose = copy.deepcopy(end_effector_pose)
#     print "test():"
#     print "= end_effector_pose:"
#     print local_end_effector_pose

#     limb_joints = get_limb_postion_via_inverse_kinematics(local_end_effector_pose, ik_seed)
#     if limb_joints == False:
#         if tries == 9:
#             print "@@@@@@@@@@@@@ Failed 9 times..."
#             return False
#         else:
#             print "Failed to find IK to go to target end effector pose no. ", tries
#             local_end_effector_pose.position.z += 0.01 * (random.random() - 0.5)
#             continue
#     else:
#         break
#     return plan_and_move_to_joints_configuration(limb_joints, speed)

def get_grasp_pose(g_decision_making_state, object_grasp_pose, object_grasp_pose_set):
    
	# load pre-computer grasps
	# loaded_file = numpy.load('../grasp_data/' + g_decision_making_state.object_name + '.npz' )
	
	# loaded_grasp = loaded_file['grasp'][0]
	# print len(loaded_grasp[g_decision_making_state.object_name])

	loaded_file = numpy.load('../grasp_data/' + g_decision_making_state.object_name + '.npz' )
	loaded_grasp = loaded_file['grasp'][0]
	num_of_grasp = len(loaded_grasp[g_decision_making_state.object_name])
	temp = object_grasp_pose
	for i in range(0,num_of_grasp):
		print i
		print_angles(loaded_grasp[g_decision_making_state.object_name][i])
		print_angles(temp)
		object_grasp_pose_temp = temp
		print_angles(temp)
		object_grasp_pose_set[i] = Pose()
		object_grasp_pose_set[i].position = temp.position
		object_grasp_pose_set[i].orientation = rotate_orientation(object_grasp_pose_temp.orientation, loaded_grasp[g_decision_making_state.object_name][i].orientation)

		move_ok = plan_and_move_end_effector_to(object_grasp_pose, 0.3, 
                                            g_bin_positions[g_decision_making_state.bin_id]['map_robot_position_2_ik_seed'])
		print_angles(object_grasp_pose_set[i])
		# raw_input("press Enter to continue...")

	return object_grasp_pose_set
