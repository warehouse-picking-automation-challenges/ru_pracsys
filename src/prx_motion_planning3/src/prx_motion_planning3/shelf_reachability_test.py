#!/usr/bin/env python

import sys 
import rospy

import math
import numpy
import time
import copy
import baxter_interface
from baxter_interface import CHECK_VERSION
import math
import tf

import std_msgs.msg
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState, Image
#from object_recognition_msgs.msg import RecognizedObjectArray

from prx_decision_making.msg import DecisionMakingStateMessage
from prx_decision_making.msg import MotionPlanningStateMessage

from baxter_core_msgs.msg import EndpointState

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

# from planning_library_interface import *
# from pracsys_library_interface import *
from motoman_pracsys_library_interface import *

# from grasping_planner import *

from prx_planning.srv import *

from dynamic_reconfigure.server import Server
#from prx_mapping.cfg import prx_mappingConfig

import roslib; roslib.load_manifest('rviz_python_tutorial')

import ast

from visualization_msgs.msg import MarkerArray,Marker
    
##############################################################################################
# Definitions                                                                                #
##############################################################################################

BAXTER = 0
MOTOMAN = 1

X_CLOSE_TO_OBJECT = 0.115
X_CLOSE_TO_OBJECT2 = 0.025
Z_CLOSE_TO_OBJECT = 0.035
X_ON_EDGE_OF_BIN = 0.7
X_APPROACH_STEP = 0.01

BAXTER_BASE_HIGH = 0.9

ROBOT_SPEED_SLOW = 0.15
ROBOT_SPEED_FAST = 0.25


##############################################################################################
# Support functions                                                                          #
##############################################################################################

def xfrange(start, stop, step):
    while start <= stop:
        yield start
        start += step
    return


def orientation_from_quaternion(quaternion):
    orientation = Quaternion()
    orientation.x = quaternion[0]
    orientation.y = quaternion[1]
    orientation.z = quaternion[2]
    orientation.w = quaternion[3]
    
    return orientation


def quaternion_from_orientation(orientation):
    return [orientation.x, orientation.y, orientation.z, orientation.w]


def position_as_list(position):
    return [position.x, position.y, position.z]


def list_as_position(list):
    position = Point()
    position.x = list[0]
    position.y = list[1]
    position.z = list[2]

    return position


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


def get_object_pose():
    global g_shelf_description
    global g_base_reference_frame
    global g_robot
    global HARD_CODED_OBJECT
    
    object_pose = Pose()
    if HARD_CODED_OBJECT == True:
        print "####@@@####@@@ ATENTION: Hard coded object pose in use ####@@@####@@@ "
        if g_robot == BAXTER:
            object_pose.position.x = 1.03316833124 #trans[0]
            object_pose.position.y = 0.55459452847 #trans[1]
            object_pose.position.z = 0.288401514479 #trans[2]
            object_pose.orientation.x = -0.477072525368 #rot[0]
            object_pose.orientation.y = 0.534384847975 #rot[1]
            object_pose.orientation.z = -0.488505960788 #rot[2]
            object_pose.orientation.w = 0.498193301911 #rot[3]
            return object_pose
        else:
            object_pose.position.x = 1.015 #trans[0]
            object_pose.position.y = -0.28 #trans[1]
            object_pose.position.z = 0.716 #trans[2]
            object_pose.orientation.x = -0.477072525368 #rot[0]
            object_pose.orientation.y = 0.534384847975 #rot[1]
            object_pose.orientation.z = -0.488505960788 #rot[2]
            object_pose.orientation.w = 0.498193301911 #rot[3]
            return object_pose
    
    for i in range(0, 10):
        object_pose = get_tf_transform(g_base_reference_frame, '/object_frame')
        if object_pose != False:
           if object_pose.position.x < g_shelf_description.x:
               print "======= Object detected outside (in front) of the bin"
               print "object_pose.position.x = ", object_pose.position.x, "   g_shelf_description.x ", g_shelf_description.x
           else:
               return object_pose
    return False


def rotate_orientation_quaternion(orientation, 
        first_rotating_axis, first_rotating_axis_signal, 
        second_rotating_axis, second_rotating_axis_signal):
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
    #print "rotation1 ========== ", rotation1
    new_orientation = tf.transformations.quaternion_multiply(_orientation, rotation1)
    #print "rotation2 ========== ", rotation2
    new_orientation = tf.transformations.quaternion_multiply(new_orientation, rotation2)
    #print first_rotating_axis, second_rotating_axis, first_rotating_axis_signal, second_rotating_axis_signal
    orientation_rotated = Quaternion()
    orientation_rotated.x = new_orientation[0]
    orientation_rotated.y = new_orientation[1]
    orientation_rotated.z = new_orientation[2]
    orientation_rotated.w = new_orientation[3]

    return orientation_rotated


def inverse_orientation(orientation):
    _orientation = numpy.empty((4, ), dtype=numpy.float64)
    _orientation[0] = orientation.x
    _orientation[1] = orientation.y
    _orientation[2] = orientation.z
    _orientation[3] = orientation.w

    new_orientation = tf.transformations.quaternion_inverse(_orientation)

    orientation_rotated = Quaternion()
    orientation_rotated.x = new_orientation[0]
    orientation_rotated.y = new_orientation[1]
    orientation_rotated.z = new_orientation[2]
    orientation_rotated.w = new_orientation[3]

    return orientation_rotated


def add_shelf_to_scene():
    global g_shelf_on_the_scene
    global g_shelf_description
    
    # add_order_bin_to_scene()
    if g_shelf_on_the_scene == False:
        print "== Adding shelf to the scene"
        shelf_pose = PoseStamped()
        shelf_pose.header = Header(stamp = rospy.Time.now(), frame_id = '/world')
        shelf_pose.pose.position.x = g_shelf_description.x + g_shelf_description.depth / 2.0 # see shelf_description.txt
        shelf_pose.pose.position.y = g_shelf_description.y
        shelf_pose.pose.position.z = g_shelf_description.z + g_shelf_description.height / 2.0 # see shelf_description.txt
        shelf_orientation = tf.transformations.quaternion_from_euler(
            g_shelf_description.roll * math.pi / 180.0, 
            g_shelf_description.pitch * math.pi / 180.0, 
            g_shelf_description.yaw * math.pi / 180.0)
        shelf_pose.pose.orientation = orientation_from_quaternion(shelf_orientation)
        print shelf_pose.pose
        add_box_to_scene("shelf", shelf_pose, (g_shelf_description.depth, g_shelf_description.width, g_shelf_description.height)) # delete the previous octomap

        rospy.sleep(3.0)
        g_shelf_on_the_scene = True
    
    return


def add_order_bin_to_scene():
    global g_order_bin_description
    
    print "== Adding order bin to the scene"
    order_bin_pose = PoseStamped()
    order_bin_pose.header = Header(stamp = rospy.Time.now(), frame_id = '/world')
    order_bin_pose.pose.position.x = g_order_bin_description.x + g_order_bin_description.depth / 2.0 # see order_bin_description.txt
    order_bin_pose.pose.position.y = g_order_bin_description.y
    order_bin_pose.pose.position.z = g_order_bin_description.z + g_order_bin_description.height / 2.0 # see order_bin_description.txt
    order_bin_orientation = tf.transformations.quaternion_from_euler(
        g_order_bin_description.roll * math.pi / 180.0, 
        g_order_bin_description.pitch * math.pi / 180.0, 
        g_order_bin_description.yaw * math.pi / 180.0)
    order_bin_pose.pose.orientation = orientation_from_quaternion(order_bin_orientation)
    add_box_to_scene("order_bin", order_bin_pose, (g_order_bin_description.depth, g_order_bin_description.width, g_order_bin_description.height))

    return


def remove_shelf_from_the_scene():
    global g_shelf_on_the_scene
    
    print "== Removing shelf from the scene"
    header = Header(stamp = rospy.Time.now(), frame_id = '/world')
    remove_object_from_scene("shelf", header)
    g_shelf_on_the_scene = False
    
    return

def print_angles(pose):
    angles = tf.transformations.euler_from_quaternion(quaternion_from_orientation(pose.orientation))
    print angles[0] * 180.0 / math.pi, ", ", angles[1] * 180.0 / math.pi, ", ", angles[2] * 180.0 / math.pi
    
    return


class FutureMotomanGripper():
    def __init__(self):
        return
    
    def open(self):
        return

    def close(self):
        return

    def reset(self):
        return

    def calibrate(self):
        return


def set_and_calibrate_gripper(gripper_name):
    global g_robot
    global g_gripper

    if g_robot == BAXTER:
        g_gripper = baxter_interface.Gripper(gripper_name)
        if g_gripper.error():
            g_gripper.reset()
        
        g_gripper.calibrate()
    else:
        g_gripper = FutureMotomanGripper()
        
    return


##############################################################################################
# Motion functions                                                                           #
##############################################################################################

def compute_grasp_pose(translation):
    global g_object_pose
    
    grasp_pose = PoseStamped()
    grasp_pose.header = Header(stamp = rospy.Time.now(), frame_id = 'world')
    # Rotate to align with gripper
    object_pose = copy.deepcopy(g_object_pose)
    grasp_pose.pose.orientation = rotate_orientation_quaternion(object_pose.orientation, 
                                                                3, 1.0, 
                                                                1, 0.0)
    # Translate according to object pose (X direction)
    t = tf.transformations.quaternion_matrix(quaternion_from_orientation(object_pose.orientation)).dot([translation, 0.0, 0.0, 0.0])
    # Away from object
    #print t[:3]
    #print object_pose.position
    # X = -Y; Y = X
    grasp_pose.pose.position.x = object_pose.position.x - t[1]
    grasp_pose.pose.position.y = object_pose.position.y + t[0]
    grasp_pose.pose.position.z = object_pose.position.z + t[2]
    #print grasp_pose.pose.position
    return grasp_pose


def move_closer_to_object():
    global g_object_pose
    global g_motion_planner_state_publisher
    global g_decision_making_state
    global g_bin_positions
    global g_base_reference_frame
    global g_robot

    if g_object_pose != False:
        pose_close_to_object = copy.deepcopy(g_object_pose)
        pose_close_to_object.orientation = g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['bin_robot_position'].orientation
        pose_close_to_object.position.x = g_object_pose.position.x - X_CLOSE_TO_OBJECT # stop close to the object
        # pose_close_to_object = compute_grasp_pose(X_CLOSE_TO_OBJECT).pose
        
        move_ok = plan_and_move_end_effector_to(g_decision_making_state.robot_arm, pose_close_to_object, ROBOT_SPEED_SLOW, g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_2_ik_seed'])
    
        if move_ok:
            state = 4 # robot is close to object
        else:
            print "######### Could not approach for grasping..."
            header = Header(stamp = rospy.Time.now(), frame_id = '/world')
            remove_object_from_scene(g_decision_making_state.object_name, header)
            state = 8 # robot fail to get close to object
    else:
        print "######### No object detected."
        state = 8 # robot fail to get close to object -> send state machine back to move to Bin

    header = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
    g_motion_planner_state_publisher.publish(header, state)
    rospy.sleep(3.0) # Alberto: Critical time. If too small (i.e. 2.0), end effector does not reach object correctly. 
    
    return state


def attach_object_to_the_gripper():
    global g_object_pose
    global g_object_attached_to_the_gripper
    global g_decision_making_state
    global first_rotating_axis
    global first_rotating_axis_signal
    global second_rotating_axis
    global second_rotating_axis_signal
    global g_base_reference_frame
    global g_robot
    
    if g_object_attached_to_the_gripper == False:
        if g_robot == BAXTER:
            if g_decision_making_state.robot_arm == 0:
                gripper_name = 'left_gripper'
            else:
                gripper_name = 'right_gripper'
        else:
            if g_decision_making_state.robot_arm == 0:
                gripper_name = '/arm_left_link_7_t'
            else:
                gripper_name = '/arm_right_link_7_t'
        attaching_pose_in_gripper = PoseStamped()
        attaching_pose_in_gripper.header = Header(stamp = rospy.Time.now(), frame_id = gripper_name)
        attaching_pose_in_gripper.pose.position = Point(*[0.0, 0.0, 0.06]) # TODO: This constant is object dependent
        
        gripper_pose_with_respect_to_base = get_tf_transform(g_base_reference_frame, gripper_name)
        attaching_pose_in_gripper.pose.orientation = rotate_orientation(gripper_pose_with_respect_to_base.orientation, 
                                                                        rotate_orientation_quaternion(g_object_pose.orientation, 
                                                                                                      first_rotating_axis, first_rotating_axis_signal, 
                                                                                                      second_rotating_axis, second_rotating_axis_signal))
        print "Attaching the object mesh to the gripper"
        attach_mesh_to_robot(g_decision_making_state.robot_arm, gripper_name, g_decision_making_state.object_name + '_attached', attaching_pose_in_gripper, g_decision_making_state.object_name + ".stl")
        g_object_attached_to_the_gripper = True
    
    rospy.sleep(0.8)
    
    return
        

def dettach_object_from_the_gripper():
    global g_object_attached_to_the_gripper
    global g_decision_making_state
    
    if g_object_attached_to_the_gripper == True:
        if g_decision_making_state.robot_arm == 0:
            remove_mesh_from_robot("left_gripper", g_decision_making_state.object_name + '_attached')
        else:
            remove_mesh_from_robot("right_gripper", g_decision_making_state.object_name + '_attached')
        rospy.sleep(2.0)
        header = Header(stamp = rospy.Time.now(), frame_id = '/world')
        remove_object_from_scene(g_decision_making_state.object_name + '_attached', header)
        g_object_attached_to_the_gripper = False
        
    return

        
def grasp():
    global g_gripper
    global g_object_pose
    global g_motion_planner_state_publisher
    global g_decision_making_state
    global g_base_reference_frame
    global g_bin_positions
    global g_robot

    print "Removing the object mesh from the scene"
    header = Header(stamp = rospy.Time.now(), frame_id = '/world')
    remove_object_from_scene(g_decision_making_state.object_name, header)

    g_gripper.open()
    rospy.sleep(0.8)

    object_grasp_pose = copy.deepcopy(g_object_pose)
    object_grasp_pose.orientation = g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['bin_robot_position'].orientation
    
    object_grasp_pose.position.x = g_object_pose.position.x - X_CLOSE_TO_OBJECT2
    move_ok = plan_and_move_end_effector_to(g_decision_making_state.robot_arm, object_grasp_pose, ROBOT_SPEED_SLOW, 
                                            g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_2_ik_seed'])
    if move_ok == True:
        g_gripper.close() # grasp!
        attach_object_to_the_gripper()
        print "################ GRASPED!!"
        
        # clear_octomap()
            
        # Move the object up
        object_grasp_pose.position.z = object_grasp_pose.position.z + Z_CLOSE_TO_OBJECT 
        print "Try and raise the object"
        move_ok = plan_and_move_end_effector_to(g_decision_making_state.robot_arm, object_grasp_pose, ROBOT_SPEED_SLOW,
                                                g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_2_ik_seed'])
        if move_ok == True:
            state = 7 # grasped and raised
        else:
            print "######### Could not raise"
            g_gripper.open()
            dettach_object_from_the_gripper()
            state = 8 # robot fail to move the object up
    else:
        print "######### Could not grasp"
        state = 8 # robot fail to get close to object
    
    header = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
    g_motion_planner_state_publisher.publish(header, state)
    rospy.sleep(1.8)
    
    return state
    

def move_away_from_bin():
    global g_object_pose
    global g_motion_planner_state_publisher
    global g_shelf_description
    global g_base_reference_frame
    global g_bin_positions
    global g_robot
    
    x_origin = g_shelf_description.x - 0.15 # TODO: this constant should be a define

    pose_away_from_bin = copy.deepcopy(g_object_pose)
    pose_away_from_bin.orientation = g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['bin_robot_position'].orientation
    pose_away_from_bin.position.x = x_origin 
    print "Try and retract the end effector with the object outside of the bin"
    move_ok = plan_and_move_end_effector_to(g_decision_making_state.robot_arm, pose_away_from_bin, ROBOT_SPEED_SLOW, g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_2_ik_seed'])
    if move_ok:
        rospy.sleep(0.8)
        print "########### Successfully grasped and retracted outside of bin"
        state = 5 # moved away from bin
    else:
        rospy.sleep(0.8)
        print "######### Could not retract #"
        g_gripper.open()
        dettach_object_from_the_gripper()
        state = 8 # robot fail to move the object up

    header = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
    g_motion_planner_state_publisher.publish(header, state)
    rospy.sleep(0.8)

    return state


##############################################################################################
# Main operating function                                                                    #
##############################################################################################

def motion_planner():
    global g_decision_making_state
    global g_decision_making_state_available
    global g_motion_planner_state_publisher
    global g_gripper
    global g_object_pose
    global g_bin_positions
    global first_rotating_axis
    global first_rotating_axis_signal
    global second_rotating_axis
    global second_rotating_axis_signal
    global g_base_reference_frame
    global g_robot

    state = 0 # none
    if g_decision_making_state_available == True:
        g_decision_making_state_available = False
        current_motion_planning_mode = g_decision_making_state.motion_planning_mode

        if current_motion_planning_mode == 6: # mp_to_O -> move to origin
            print "============== Moving to origin =============="
            add_shelf_to_scene()
            
            g_gripper.close()
            
            print "=== Current bin: ", g_decision_making_state.bin_id
            move_ok = False
            while move_ok == False:
                move_ok = plan_and_move_to_joints_configuration(g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['origin_robot_configuration'], ROBOT_SPEED_FAST)
            state = 1 # robot is at the origin
            header = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
            g_motion_planner_state_publisher.publish(header, state)
            rospy.sleep(0.8)

        if current_motion_planning_mode == 7: # mp_to_B -> move to Bin
            print "============== Moving to bin =============="
            # print g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['bin_robot_position']
            # plan_and_move_end_effector_to(g_decision_making_state.robot_arm, g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['bin_robot_position'], ROBOT_SPEED_FAST, 
            #    g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_2_ik_seed'])
            move_ok = False
            while move_ok == False:
                move_ok = plan_and_move_end_effector_to(g_decision_making_state.robot_arm, g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_2'], ROBOT_SPEED_FAST, 
                                              g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_2_ik_seed'])
            add_shelf_to_scene()    

            rospy.sleep(2.0)
            clear_octomap()
            
            state = 2 # robot is at the bin
            header = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
            g_motion_planner_state_publisher.publish(header, state)
            rospy.sleep(0.8)

        if current_motion_planning_mode == 8: # mp_to_M -> move to map
            print "============== Mapping and Object Detection =============="
            rospy.sleep(1.0)
            print "Removing shelf model"
            remove_shelf_from_the_scene()
            print "Shelf model removed"
            rospy.sleep(1.0)
            
            # plan_and_move_end_effector_to(g_decision_making_state.robot_arm, g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_1'], ROBOT_SPEED_FAST, 
            #    g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_2_ik_seed'])
            # rospy.sleep(1.0)
            # plan_and_move_end_effector_to(g_decision_making_state.robot_arm, g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_2'], ROBOT_SPEED_FAST
            #    g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['map_robot_position_2_ik_seed'])
        
            print "Get object pose"
            g_object_pose = False
            while g_object_pose == False:
                g_object_pose = get_object_pose()
                
            object_mesh_pose = PoseStamped()
            object_mesh_pose.header = Header(stamp = rospy.Time.now(), frame_id = '/world')
            object_mesh_pose.pose = g_object_pose
            object_mesh_pose.pose.orientation = rotate_orientation_quaternion(object_mesh_pose.pose.orientation, 
                                                                              first_rotating_axis, first_rotating_axis_signal, 
                                                                              second_rotating_axis, second_rotating_axis_signal)
            print "Adding the object mesh to the scene"  
            add_mesh_to_scene(g_decision_making_state.object_name, object_mesh_pose, g_decision_making_state.object_name + ".stl")
            print "Object model added"
            
            clear_octomap()
        
            rospy.sleep(3.0)

            state = 3 # bin mapped
            header = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
            g_motion_planner_state_publisher.publish(header, state)
            rospy.sleep(0.8)

        if current_motion_planning_mode == 9: # mp_to_Ob_ -> move closer to object
            print "============== Moving close to object =============="
            state = move_closer_to_object()

        if current_motion_planning_mode == 12: # grasp -> grasp move
            print "============== Grasping =============="
            state = grasp()
    
        if current_motion_planning_mode == 10: # mp_to_AB_ -> move away from bin
            print "============== Moving away from bin =============="
            state = move_away_from_bin()

        if current_motion_planning_mode == 11: # mp_to_OB -> move to order bin
            print "============== Moving to order bin =============="
            add_shelf_to_scene()

            move_ok = False
            while move_ok == False:
                move_ok = plan_and_move_to_joints_configuration(g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['order_bin_robot_configuration'], ROBOT_SPEED_FAST)
            rospy.sleep(0.5)
            g_gripper.open()
            dettach_object_from_the_gripper()
            # clear_octomap()

            rospy.sleep(1.0)
            move_ok = False
            while move_ok == False:
                move_ok = plan_and_move_to_joints_configuration(g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['origin_robot_configuration'], ROBOT_SPEED_FAST)

            state = 6 # robot is at the order bin
            header = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
            g_motion_planner_state_publisher.publish(header, state)
            rospy.sleep(0.8)
             
        print "=== motion planner new state = ", state, "decision making state = ", g_decision_making_state.state, "current motion planning mode = ", current_motion_planning_mode

    print "@@@ === motion planner new state = ", state, "decision making state = ", g_decision_making_state.state

    return
    

##############################################################################################
# Handlers                                                                                   #
##############################################################################################

def camera_image_color_handler(data):
    global g_decision_making_state
    global g_decision_making_previous_state
    global g_camera_image_color_update
    
    if g_decision_making_state.robot_arm != 0: # it is not the left arm
        return
    
    if g_camera_image_color_update <= 0 and g_decision_making_previous_state != None:
        motion_planner()
        g_camera_image_color_update = 5
    else:
        g_camera_image_color_update -= 1
    
    return


def camera2_image_color_handler(data):
    global g_decision_making_state
    global g_decision_making_previous_state
    global g_camera_image_color_update
    
    if g_decision_making_state.robot_arm != 1: # it is not the right arm
        return
    
    if g_camera_image_color_update <= 0 and g_decision_making_previous_state != None:
        motion_planner()
        g_camera_image_color_update = 5
    else:
        g_camera_image_color_update -= 1
    
    return


def decision_making_state_handler(data):
    global g_decision_making_state
    global g_decision_making_state_available
    global g_tf_transform_broadcaster
    global g_bin_positions
    global g_decision_making_previous_state
    global g_previous_arm
    global g_base_reference_frame
    global g_robot
    global g_move_group_name
    
    g_decision_making_state = data
    g_decision_making_state.bin_id = chr(g_decision_making_state.bin_id)
    
    if g_decision_making_state_available == False:
        if g_decision_making_previous_state != None:
            if g_decision_making_state.state != g_decision_making_previous_state.state:
                g_decision_making_state_available = True
        else:
            g_decision_making_state_available = True
        g_decision_making_previous_state = g_decision_making_state

    # Publish objec_pose_hitX tf transforms
    orientation = quaternion_from_orientation(g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['object_pose_hint0'].orientation)
    position = position_as_list(g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['object_pose_hint0'].position)
    g_tf_transform_broadcaster.sendTransform(position, orientation, rospy.Time.now(), "/object_pose_hint0", g_base_reference_frame)
    position = position_as_list(g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['object_pose_hint1'].position)
    g_tf_transform_broadcaster.sendTransform(position, orientation, rospy.Time.now(), "/object_pose_hint1", g_base_reference_frame)
    position = position_as_list(g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['object_pose_hint2'].position)
    g_tf_transform_broadcaster.sendTransform(position, orientation, rospy.Time.now(), "/object_pose_hint2", g_base_reference_frame)
    position = position_as_list(g_bin_positions[g_robot][g_decision_making_state.robot_arm][g_decision_making_state.bin_id]['object_pose_hint3'].position)
    g_tf_transform_broadcaster.sendTransform(position, orientation, rospy.Time.now(), "/object_pose_hint3", g_base_reference_frame)

    if g_decision_making_state.robot_arm != g_previous_arm:
        g_previous_arm = g_decision_making_state.robot_arm
        if g_decision_making_state.robot_arm == 0:
            set_and_calibrate_gripper('left')
            if g_robot == BAXTER:
                init_planning_library('left_arm', g_robot)
            else:
                init_planning_library('arm_left', g_robot)
        else:
            set_and_calibrate_gripper('right')
            if g_robot == BAXTER:
                init_planning_library('right_arm', g_robot)
            else:
                init_planning_library('arm_right', g_robot)
    
    return

    
def endpoint_state_state_handler(data):
    global g_endpoint_state
    global g_endpoint_state_available

    g_endpoint_state = data
    g_endpoint_state_available = True
    
    return
 
    
def baxter_joints_state_handler(data):
    global g_joints_state_update
    global g_decision_making_state
    global g_decision_making_previous_state
    
    if g_joints_state_update <= 0 and g_decision_making_previous_state != None:
        left_joints_state = {'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0, 'left_s0': 0.0, 'left_s1': 0.0}
        for key in left_joints_state.keys():
            left_joints_state[key] = data.position[data.name.index(key)]

        right_joints_state = {'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0, 'right_e0': 0.0, 'right_e1': 0.0, 'right_s0': 0.0, 'right_s1': 0.0}
        for key in right_joints_state.keys():
            right_joints_state[key] = data.position[data.name.index(key)]
        
        if g_decision_making_state.robot_arm == 0: # left
            update_planning_library_state(left_joints_state)
        else:
            update_planning_library_state(right_joints_state)
        
        g_joints_state_update = 10
    else:
        g_joints_state_update -= 1

    return
 
    
def motoman_joints_state_handler(data):
    global g_joints_state_update
    global g_decision_making_state
    global g_decision_making_previous_state
    global g_joints_state
    
    if g_joints_state_update <= 0 and g_decision_making_previous_state != None:
        for key in data.name:
            g_joints_state[key] = data.position[data.name.index(key)]

        update_planning_library_state(g_joints_state)        
        g_joints_state_update = 5
    else:
        g_joints_state_update -= 1

    return

    
def detected_object_handler(data):
    global g_detected_object_array
    
    g_detected_object_array = data

    #print g_detected_object_array.objects[0].confidence
    #print g_detected_object_array.objects[0].bounding_mesh.vertices[0]
    #print g_detected_object_array.objects[0].point_clouds[0]
    
    return


def reconfigure_callback(config, level):
    global first_rotating_axis
    global first_rotating_axis_signal
    global second_rotating_axis
    global second_rotating_axis_signal
    
    first_rotating_axis = config.first_rotating_axis
    second_rotating_axis = config.second_rotating_axis
    first_rotating_axis_signal = config.first_rotating_axis_signal
    second_rotating_axis_signal = config.second_rotating_axis_signal
    
    print "==== Rotations updated"
    print first_rotating_axis, second_rotating_axis, first_rotating_axis_signal, second_rotating_axis_signal
    
    return config
    

##############################################################################################
# Initializations                                                                            #
##############################################################################################

class ShelfDescription(object):
    def __init__(self, initial_data):
       for key in initial_data:
           setattr(self, key, initial_data[key])
    
    def __str__(self):
        return "{'x':%s, 'y':%s, 'z':%s, 'roll':%s, 'pitch':%s, 'yaw':%s, 'width':%s, 'height':%s, 'rows':%s, 'cols':%s, 'a_col_width':%s, 'b_col_width':%s, 'c_col_width':%s, 'a_row_height':%s, 'd_row_height':%s, 'g_row_height':%s, 'j_row_height':%s, 'shelfs_lip_height':%s, 'support_column_width':%s}"  % (
                self.x, self.y, self.z, self.roll, self.pitch, self.yaw, 
                self.width, self.height, self.rows, self.cols, 
                self.a_col_width, self.b_col_width, self.c_col_width, 
                self.a_row_height, self.d_row_height, self.g_row_height, self.j_row_height, 
                self.shelfs_lip_height, self.support_column_width)


def init_shelf_parameters():  
    global g_shelf_description
    global g_robot

    if g_robot == BAXTER:
        f = open('shelf_description.txt', 'r')
    else:
        f = open('shelf_description_motoman.txt', 'r')
    str_desc = f.readline()
    g_shelf_description = ShelfDescription(ast.literal_eval(str_desc))
    f.close()
    print "Shelf description: ", g_shelf_description

    return
    

class OrderBinDescription(object):
    def __init__(self, initial_data):
       for key in initial_data:
           setattr(self, key, initial_data[key])
    
    def __str__(self):
        return "{'x':%s, 'y':%s, 'z':%s, 'roll':%s, 'pitch':%s, 'yaw':%s, 'width':%s, 'height':%s}"  % (self.x, self.y, self.z, self.roll, self.pitch, self.yaw, self.width, self.height)
    

def init_order_bin_parameters():  
    global g_order_bin_description

    if g_robot == BAXTER:
        f = open('order_bin_description.txt', 'r')
    else:
        f = open('order_bin_description_motoman.txt', 'r')
    str_desc = f.readline()
    g_order_bin_description = OrderBinDescription(ast.literal_eval(str_desc))
    f.close()
    print "Order bin descrition:", g_order_bin_description

    return


def get_z_coordinate_of_the_center_of_the_bin(bin):
    global g_shelf_description

    a_row = ['A', 'B', 'C']
    d_row = ['D', 'E', 'F']
    g_row = ['G', 'H', 'I']
    j_row = ['J', 'K', 'L']
    if a_row.count(bin) != 0: # bin is in the a row
        z_center_bin = g_shelf_description.z + 4*g_shelf_description.shelfs_lip_height+\
        g_shelf_description.j_row_height + g_shelf_description.g_row_height + g_shelf_description.d_row_height + \
        (g_shelf_description.a_row_height ) / 2.0
        bin_height = g_shelf_description.a_row_height
    elif d_row.count(bin) != 0: # bin is in the d row
        z_center_bin = g_shelf_description.z + 3*g_shelf_description.shelfs_lip_height+\
        g_shelf_description.j_row_height + g_shelf_description.g_row_height + \
        (g_shelf_description.d_row_height) / 2.0
        bin_height = g_shelf_description.d_row_height
    elif g_row.count(bin) != 0: # bin is in the g row
        z_center_bin = g_shelf_description.z + 2*g_shelf_description.shelfs_lip_height + \
        g_shelf_description.j_row_height + \
        (g_shelf_description.g_row_height ) / 2.0
        bin_height = g_shelf_description.g_row_height
    else: # bin is in the j row
        z_center_bin = g_shelf_description.z  + g_shelf_description.shelfs_lip_height + \
        (g_shelf_description.j_row_height) / 2.0
        bin_height = g_shelf_description.j_row_height
    
    return z_center_bin, bin_height


def get_y_coordinate_of_the_center_of_the_bin(bin):
    global g_shelf_description

    a_col = ['A', 'D', 'G', 'J']
    b_col = ['B', 'E', 'H', 'K']
    c_col = ['C', 'F', 'I', 'L']
    y_center_bin = 0.0
    if a_col.count(bin) != 0: # bin is in the a column
        y_center_bin = g_shelf_description.y + \
            g_shelf_description.b_col_width / 2.0 + \
            g_shelf_description.support_column_width +\
            (g_shelf_description.a_col_width) / 2.0
        bin_width = (g_shelf_description.a_col_width)
    elif b_col.count(bin) != 0: # bin is in the b column
        y_center_bin = g_shelf_description.y
        bin_width = g_shelf_description.b_col_width
    else: # bin is in the c column
        y_center_bin = g_shelf_description.y - \
        g_shelf_description.b_col_width / 2.0 - \
        g_shelf_description.support_column_width- \
        (g_shelf_description.c_col_width) / 2.0
        bin_width = g_shelf_description.c_col_width
    
    return y_center_bin, bin_width


def compute_scene_poses():
    global g_bin_positions
    global g_shelf_description
    
    g_bin_positions = {}
    for robot in 0, 1: # 0 = BAXTER, 1 = MOTOMAN
        g_bin_positions[robot] = {}
        if robot == BAXTER:
            x_origin = g_shelf_description.x - 0.15
            x_mapping = g_shelf_description.x - 0.10
            for arm in 0, 1: # 0 = left, 1 = right
                g_bin_positions[robot][arm] = {}
                for bin in 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L':
                    g_bin_positions[robot][arm][bin] = {}
                    pose = Pose()
                    pose.position.x = x_origin
                    pose.position.y = g_shelf_description.y
                    pose.position.z = g_shelf_description.z + g_shelf_description.height / 2.0
                    pose.orientation.x = math.sqrt(0.5)
                    pose.orientation.y = 0.0
                    pose.orientation.z = math.sqrt(0.5)
                    pose.orientation.w = 0.0
                    # g_bin_positions[robot][arm][bin]['origin_robot_position'] = pose
                    if arm == 0: # left
                        g_bin_positions[robot][arm][bin]['origin_robot_configuration'] = {'left_w0': -2.8516702814208985, 'left_w1': 0.43219908649291994, 'left_w2': 1.6333060420349121, 'left_e0': -1.3809662027160645, 'left_e1': 2.089665325909424, 'left_s0': 0.8013884611938477, 'left_s1': 0.1852281799255371}
                        g_bin_positions[robot][arm][bin]['order_bin_robot_configuration'] = {'left_w0': -1.3150050289123536, 'left_w1': -1.2762720140625001, 'left_w2': -0.6630631948059083, 'left_e0': -1.4787574778320314, 'left_e1': 0.7646894218872071, 'left_s0': -0.5748592996032715, 'left_s1': 0.37544179740600586}
                    else:
                        g_bin_positions[robot][arm][bin]['origin_robot_configuration'] = {'right_s0': -1.0181797467956544, 'right_s1': 0.2197427476135254, 'right_w0': 2.8002819250854496, 'right_w1': 0.34322820089721684, 'right_w2': -1.5312963197570801, 'right_e0': 1.3479856158142092, 'right_e1': 2.0904323163024903}
                        g_bin_positions[robot][arm][bin]['order_bin_robot_configuration'] = {'right_s0': 0.5069806498168946, 'right_s1': 0.08858739039916992, 'right_w0': 1.802427423706055, 'right_w1': -1.045791400946045, 'right_w2': 0.6826214498291016, 'right_e0': 1.0074418812927246, 'right_e1': 0.9487671162231446}
                    y_center_bin, bin_width = get_y_coordinate_of_the_center_of_the_bin(bin)
                    z_center_bin, bin_height = get_z_coordinate_of_the_center_of_the_bin(bin)
            
                    pose_i = copy.deepcopy(pose)
                    pose_i.position.x = x_mapping
                    pose_i.position.y = y_center_bin
                    pose_i.position.z = z_center_bin
                    g_bin_positions[robot][arm][bin]['bin_robot_position'] = pose_i
            
                    pose_j = copy.deepcopy(pose_i)
                    g_bin_positions[robot][arm][bin]['map_robot_position_1'] = pose_j
            
                    pose_k = copy.deepcopy(pose_i)
                    pose_k.position.z -= 0.05
                    g_bin_positions[robot][arm][bin]['map_robot_position_2'] = pose_k
                    g_bin_positions[robot][arm][bin]['map_robot_position_2_ik_seed'] = g_bin_positions[robot][arm][bin]['origin_robot_configuration']
            
                    pose_h0 = copy.deepcopy(pose_i)
                    pose_h0.position.x = g_shelf_description.x
                    pose_h0.position.y += bin_width / 2.0
                    pose_h0.position.z -= bin_height / 2.0
                    g_bin_positions[robot][arm][bin]['object_pose_hint0'] = pose_h0
            
                    pose_h1 = copy.deepcopy(pose_i)
                    pose_h1.position.x = g_shelf_description.x
                    pose_h1.position.y += bin_width / 2.0
                    pose_h1.position.z += bin_height / 2.0
                    g_bin_positions[robot][arm][bin]['object_pose_hint1'] = pose_h1
            
                    pose_h2 = copy.deepcopy(pose_i)
                    pose_h2.position.x = g_shelf_description.x
                    pose_h2.position.y -= bin_width / 2.0
                    pose_h2.position.z += bin_height / 2.0
                    g_bin_positions[robot][arm][bin]['object_pose_hint2'] = pose_h2
            
                    pose_h3 = copy.deepcopy(pose_i)
                    pose_h3.position.x = g_shelf_description.x
                    pose_h3.position.y -= bin_width / 2.0
                    pose_h3.position.z -= bin_height / 2.0
                    g_bin_positions[robot][arm][bin]['object_pose_hint3'] = pose_h3
        else: # robot = MOTOMAN
            x_origin = g_shelf_description.x - 0.15
            x_mapping = g_shelf_description.x - 0.30
            x_grasp = g_shelf_description.x + .02
            for arm in 0, 1: # 0 = left, 1 = right
                g_bin_positions[robot][arm] = {}
                for bin in 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L':
                    g_bin_positions[robot][arm][bin] = {}
                    pose = Pose()
                    pose.position.x = x_origin
                    pose.position.y = g_shelf_description.y
                    pose.position.z = g_shelf_description.z + g_shelf_description.height / 2.0
                    # pose.orientation.x = 0.5
                    # pose.orientation.y = -0.5
                    # pose.orientation.z = -0.5
                    # pose.orientation.w = 0.5
                    pose.orientation.x = 0
                    pose.orientation.y = math.sqrt(0.5)
                    pose.orientation.z = 0
                    pose.orientation.w = math.sqrt(0.5)
                    # g_bin_positions[robot][arm][bin]['origin_robot_position'] = pose
                    if arm == 0: # left
                        g_bin_positions[robot][arm][bin]['origin_robot_configuration'] = {'torso_joint_b1': 0, 'arm_left_joint_1_s': 1.57,'arm_left_joint_2_l': 0,'arm_left_joint_3_e': 0,'arm_left_joint_4_u': -2.09,'arm_left_joint_5_r': 0,'arm_left_joint_6_b': 0,'arm_left_joint_7_t': 0}
                        g_bin_positions[robot][arm][bin]['order_bin_robot_configuration'] = {'torso_joint_b1': 0, 'arm_left_joint_1_s': 1.396,'arm_left_joint_2_l': 1.57,'arm_left_joint_3_e': 0,'arm_left_joint_4_u': -0.2,'arm_left_joint_5_r': 0,'arm_left_joint_6_b': 0,'arm_left_joint_7_t': 0}
                    else:
                        g_bin_positions[robot][arm][bin]['origin_robot_configuration'] = {'torso_joint_b1': 0, 'arm_right_joint_1_s': 1.57,'arm_right_joint_2_l': 0,'arm_right_joint_3_e': 0,'arm_right_joint_4_u': -2.09,'arm_right_joint_5_r': 0,'arm_right_joint_6_b': 0,'arm_right_joint_7_t': 0}
                        g_bin_positions[robot][arm][bin]['order_bin_robot_configuration'] = {'torso_joint_b1': 0, 'arm_right_joint_1_s': 1.396,'arm_right_joint_2_l': 1.57,'arm_right_joint_3_e': 0,'arm_right_joint_4_u': -0.2,'arm_right_joint_5_r': 0,'arm_right_joint_6_b': 0,'arm_right_joint_7_t': 0}
                    
                    y_center_bin, bin_width = get_y_coordinate_of_the_center_of_the_bin(bin)
                    z_center_bin, bin_height = get_z_coordinate_of_the_center_of_the_bin(bin)
            
                    pose_i = copy.deepcopy(pose)
                    pose_i.position.x = x_mapping
                    pose_i.position.y = y_center_bin
                    pose_i.position.z = z_center_bin
                    g_bin_positions[robot][arm][bin]['bin_robot_position'] = pose_i
            
                    # pose_j = copy.deepcopy(pose_i)
                    # g_bin_positions[robot][arm][bin]['map_robot_position_1'] = pose_j
            
                    # pose_k = copy.deepcopy(pose_i)
                    # pose_k.position.z -= 0.05
                    # g_bin_positions[robot][arm][bin]['map_robot_position_2'] = pose_k
                    # g_bin_positions[robot][arm][bin]['map_robot_position_2_ik_seed'] = g_bin_positions[robot][arm][bin]['origin_robot_configuration']
            
                    pose_h0 = copy.deepcopy(pose_i)
                    pose_h0.position.x = g_shelf_description.x
                    pose_h0.position.y += bin_width / 2.0
                    pose_h0.position.z -= bin_height / 2.0
                    g_bin_positions[robot][arm][bin]['object_pose_hint0'] = pose_h0
                    print bin+" bottom left pose:"
                    print pose_h0
            
                    pose_h1 = copy.deepcopy(pose_i)
                    pose_h1.position.x = g_shelf_description.x
                    pose_h1.position.y += bin_width / 2.0
                    pose_h1.position.z += bin_height / 2.0
                    g_bin_positions[robot][arm][bin]['object_pose_hint1'] = pose_h1
                    print bin+" top left pose:"
                    print pose_h1
            
                    pose_h2 = copy.deepcopy(pose_i)
                    pose_h2.position.x = g_shelf_description.x
                    pose_h2.position.y -= bin_width / 2.0
                    pose_h2.position.z += bin_height / 2.0
                    g_bin_positions[robot][arm][bin]['object_pose_hint2'] = pose_h2
                    print bin+" top right pose:"
                    print pose_h2
            
                    pose_h3 = copy.deepcopy(pose_i)
                    pose_h3.position.x = g_shelf_description.x
                    pose_h3.position.y -= bin_width / 2.0
                    pose_h3.position.z -= bin_height / 2.0
                    g_bin_positions[robot][arm][bin]['object_pose_hint3'] = pose_h3
                    print bin+" bottom right pose:"
                    print pose_h3

                    # val = input("Next!!!!!!")

                    # grasp_pose = copy.deepcopy(pose_i)
                    # grasp_pose.position.x = x_grasp
                    # grasp_pose.position.y = y_center_bin
                    # grasp_pose.position.z = z_center_bin
                    # g_bin_positions[robot][arm][bin]['grasp_pose'] = grasp_pose

    return


def get_robot_name():
    global g_robot
    
    if rospy.has_param('/robot_name'):
        robot_name = rospy.get_param("/robot_name")
        if robot_name == "baxter":
            g_robot = BAXTER;
        elif robot_name == "motoman":
            g_robot = MOTOMAN;
        else:
            print "Error: unkown robot named ", robot_name
            sys.exit()
        print "robot name: ", robot_name
    else:
        print "No robot name specified. So, we will use baxter configuration."
        g_robot = BAXTER;

    return


def get_hard_coded_object():
    global HARD_CODED_OBJECT
    
    if rospy.has_param('/hard_coded_object'):
        HARD_CODED_OBJECT = rospy.get_param("/hard_coded_object")
    else:
        HARD_CODED_OBJECT = False

    print "HARD_CODED_OBJECT: ", HARD_CODED_OBJECT

    return


def motion_planner_init():
    global g_tf_transform_broadcaster
    global g_motion_planner_state_publisher
    global g_decision_making_state
    global g_decision_making_state_available
    global g_endpoint_state
    global g_endpoint_state_available
    global g_test_endpoint_state_available
    global g_shelf_on_the_scene
    global g_object_attached_to_the_gripper
    global g_decision_making_previous_state
    global g_origin_robot_position
    global g_joints_state_update
    global g_camera_image_color_update
    global g_previous_arm
    global g_robot
    global g_base_reference_frame
    global g_move_group_name
    global g_joints_state
    
    g_joints_state = {'torso_joint_b1': 0, 
                      'arm_left_joint_1_s': 0,'arm_left_joint_2_l': 0,'arm_left_joint_3_e': 0,
                      'arm_left_joint_4_u': 0,'arm_left_joint_5_r': 0,'arm_left_joint_6_b': 0,'arm_left_joint_7_t': 0,
                      'arm_right_joint_1_s': 0,'arm_right_joint_2_l': 0,'arm_right_joint_3_e': 0,
                      'arm_right_joint_4_u': 0,'arm_right_joint_5_r': 0,'arm_right_joint_6_b': 0,'arm_right_joint_7_t': 0}
    g_previous_arm = -1 # none
    g_joints_state_update = 0
    g_camera_image_color_update = 0
    g_motion_planner_state_publisher = False    
    g_decision_making_state = None
    g_decision_making_state_available = False
    g_decision_making_previous_state = None
    g_endpoint_state = False
    g_endpoint_state_available = False
    g_test_endpoint_state_available = False
    g_shelf_on_the_scene = False
    g_object_attached_to_the_gripper = False

    get_hard_coded_object()
    get_robot_name()
    if g_robot == BAXTER:
        g_base_reference_frame = "/base"
        g_move_group_name = "left_arm"
    else: # Motoman
        g_base_reference_frame = "/base_link"
        g_move_group_name = "arm_left"

    init_shelf_parameters()
    init_order_bin_parameters()
    compute_scene_poses()
    g_tf_transform_broadcaster = tf.TransformBroadcaster()

    return


def motion_planner_timer(timer_event):
    motion_planner()
    
    return

def random_sample(seed_pose,angle=0):
    pose = copy.deepcopy(seed_pose)
    offset = .03
    # pose.position.x = random.uniform(.3,.55);
    pose.position.x = .48
    pose.position.y = random.uniform(pose.position.y-offset,pose.position.y+offset);
    pose.position.z = random.uniform(pose.position.z-offset,pose.position.z+offset);
    sampled_angle = random.uniform(-angle,angle)
    axis_x = math.sin(sampled_angle)
    axis_y = math.cos(sampled_angle)
    axis_z = 0
    sampled_angle = random.uniform(1.57-angle,angle+1.57)/2.0
    pose.orientation.x = axis_x*math.sin(sampled_angle)
    pose.orientation.y = axis_y*math.sin(sampled_angle)
    pose.orientation.z = axis_z*math.sin(sampled_angle)
    pose.orientation.w = math.cos(sampled_angle)
    
    return pose

import numpy
def motion_planner_main():
    global g_motion_planner_state_publisher

    topic = '/visualization_marker'
    publisher = rospy.Publisher(topic, Marker)
    rospy.init_node('motion_planner', anonymous=True)
    
    read_command_line()
    init_planning_library("arm_left","moto")

    # Initializations
    motion_planner_init()

    # print g_bin_positions[1][1]['L']['bin_robot_position']

    g_to_origin_robot_position = {'arm_left_joint_1_s': 1.57,'arm_left_joint_2_l': 0,'arm_left_joint_3_e': 0,'arm_left_joint_4_u': -1.7,'arm_left_joint_5_r': 0,'arm_left_joint_6_b': 0,'arm_left_joint_7_t': 0,'arm_right_joint_1_s': 1.57,'arm_right_joint_2_l': 0,'arm_right_joint_3_e': 0,'arm_right_joint_4_u': -1.7,'arm_right_joint_5_r': 0,'arm_right_joint_6_b': 0,'arm_right_joint_7_t': 0,'torso_joint_b1':0,'head_hinge':0}
    g_to_order_bin_robot_position = {'arm_left_joint_1_s': 1.396,'arm_left_joint_2_l': 1.57,'arm_left_joint_3_e': 0,'arm_left_joint_4_u': 0.1745,'arm_left_joint_5_r': 0,'arm_left_joint_6_b': 0,'arm_left_joint_7_t': 0,'arm_right_joint_1_s': 1.396,'arm_right_joint_2_l': 1.57,'arm_right_joint_3_e': 0,'arm_right_joint_4_u': 0.1745,'arm_right_joint_5_r': 0,'arm_right_joint_6_b': 0,'arm_right_joint_7_t': 0,'torso_joint_b1':0,'head_hinge':0}


    # markerArray = MarkerArray()

    # count = 900000
    # for bin in 'A','B','C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L':
    #     marker = Marker()
    #     marker.header.frame_id = "/base_link"
    #     marker.type = marker.SPHERE
    #     marker.action = marker.ADD
    #     marker.frame_locked = True
    #     marker.ns = rospy.get_name()
    #     # marker.header.frame_id = "bin_"+bin;
    #     marker.header.stamp =  rospy.Time.now();
    #     marker.lifetime = rospy.Duration(300);
    #     marker.id = count
    #     count = count +1
    #     marker.scale.x = 1
    #     marker.scale.y = 1
    #     marker.scale.z = 1
    #     marker.color.a = 1.0
    #     marker.color.r = 1.0
    #     marker.color.g = 1.0
    #     marker.color.b = 1.0
    #     marker.pose = g_bin_positions[1][1][bin]['bin_robot_position'];
    #     # markerArray.markers.append(marker)
    #     publisher.publish(marker)


    # return
    speed = 1
    init_planning_library("arm_right","moto")
    plan_and_move_to_joints_configuration(g_to_origin_robot_position,speed)
    # return;
    init_planning_library("arm_left","moto")
    plan_and_move_to_joints_configuration(g_to_origin_robot_position,speed)

    name = input("Next ")

    add_shelf_to_scene()
    # while True:
    #     hand = "arm_right"
    #     # for hand in "arm_right","arm_left":
    #     init_planning_library(hand,"moto")
    #     for bin in 'A','B','C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L':
    #         sampled_pose = random_sample(g_bin_positions[1][1][bin]['bin_robot_position'],0);
    #         # print sampled_pose
    #         res = get_limb_postion_via_inverse_kinematics(sampled_pose,0)
    #         # res = plan_and_move_end_effector_to("",sampled_pose,1,0)
    #         name = input("Next? ")
    #         # plan_and_move_to_joints_configuration(g_to_origin_robot_position,speed)

    # return;

    bins = numpy.linspace(.47,.49, 2)
    successful_runs = {}
    failed_runs = {}
    for hand in "arm_right","arm_left":
        successful_runs[hand] = {}
        failed_runs[hand] = {}
        init_planning_library(hand,"moto")
        for bin in 'A','B','C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L':
        # for bin in 'G', 'H', 'I':
            successful_runs[hand][bin] = []
            failed_runs[hand][bin] = []
            for val in xrange(100):
                sampled_pose = random_sample(g_bin_positions[1][1][bin]['bin_robot_position'],0);
                # print sampled_pose
                res = get_limb_postion_via_inverse_kinematics_camera(sampled_pose,0)
                if res != False:
                    successful_runs[hand][bin].append(sampled_pose.position.x);
                else:
                    failed_runs[hand][bin].append(sampled_pose.position.x);

            failed = numpy.histogram(failed_runs[hand][bin], bins);
            succeeded = numpy.histogram(successful_runs[hand][bin], bins);

            result = []
            for i in xrange(len(failed[0])):
                result.append(succeeded[0][i]*1.0/(succeeded[0][i]+failed[0][i]))

            f = open(hand+"_"+bin+".txt", 'w')
            for item in result:
                print result
                f.write(str(item)+"\n");
            print "Done "+bin+"---------------------------------------"
            f.close()
            f = open("bins.txt", 'w')
            for item in failed[1]:
                f.write(str(item)+"\n");
            f.close()
        break


if __name__ == '__main__':
    motion_planner_main()
