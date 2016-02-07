#!/usr/bin/env python

import sys 
import rospy
import copy
import math

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from std_msgs.msg import Header
import std_msgs.msg
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, RobotTrajectory, RobotState, Constraints, JointConstraint, OrientationConstraint, PositionConstraint, VisibilityConstraint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import *

import random

BAXTER = 0
MOTOMAN = 1

MOVE_CLOSER_CONSTRAINTS = 1
GRASP_CONSTRAINTS = 2
MOVE_AWAY_CONSTRAINTS = 4
MAP_CONSTRAINTS = 5


def get_limb_position_via_inverse_kinematics_baxter(robot_arm, endpoint_desired_state, ik_seed):
    global g_base_reference_frame

    if robot_arm == 0: # left
        ns = "ExternalTools/left/PositionKinematicsNode/IKService"
    else:
        ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
    
    pose = PoseStamped(hdr, endpoint_desired_state)
    ikreq.pose_stamp.append(pose)
    ikreq.seed_mode = 0 # SEED_AUTO = 0, SEED_CURRENT = 2, see http://sdk.rethinkrobotics.com/wiki/API_Reference#Inverse_Kinematics_Solver_Service
    js = JointState()
    js.header = hdr
    js.name = ik_seed.keys()
    js.position = ik_seed.values()
    ikreq.seed_angles.append(js)
    # print ikreq
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("######### Service call failed: %s" % (e,))
        sys.exit(0)
    if (resp.isValid[0]):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    else:
        print("Could not find IK for given pose in get_limb_position_via_inverse_kinematics()")
        limb_joints = False
    
    return limb_joints


def get_joints_positions_from_joint_state(joint_state):
    global g_joints_state
    
    joints_positions = g_joints_state # just to get the joints names
    for i in range(len(joint_state.name)):
        joint_name = joint_state.name[i]
        if joint_name in joints_positions.keys():
            joints_positions[joint_name] = joint_state.position[i]

    return joints_positions


def set_bin_center_pose(pose):
    global g_bin_center_pose
    
    g_bin_center_pose = pose
    
    return


def get_end_effector_constraint(constraints_set):
    global g_bin_center_pose
    global g_base_reference_frame
    
    constraints = Constraints()
    if constraints_set == MOVE_CLOSER_CONSTRAINTS:
        constraints.name = "MOVE_CLOSER_CONSTRAINTS"
        joint_constraint = JointConstraint()    
        joint_constraint.joint_name = 'head_hinge'
        joint_constraint.position = 0.1
        joint_constraint.tolerance_above = 0.9
        joint_constraint.tolerance_below = 0.1
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    elif constraints_set == GRASP_CONSTRAINTS:
        constraints.name = "GRASP_CONSTRAINTS"
        joint_constraint = JointConstraint()    
        joint_constraint.joint_name = 'head_hinge'
        joint_constraint.position = 0.1
        joint_constraint.tolerance_above = 0.9
        joint_constraint.tolerance_below = 0.1
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    elif constraints_set == MOVE_AWAY_CONSTRAINTS:
        constraints.name = "MOVE_AWAY_CONSTRAINTS"
        joint_constraint = JointConstraint()    
        joint_constraint.joint_name = 'head_hinge'
        joint_constraint.position = 0.1
        joint_constraint.tolerance_above = 0.9
        joint_constraint.tolerance_below = 0.1
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    elif constraints_set == MAP_CONSTRAINTS:
        constraints.name = "MAP_CONSTRAINTS"
        
        joint_constraint = JointConstraint()    
        joint_constraint.joint_name = 'head_hinge'
        joint_constraint.position = 0.0
        joint_constraint.tolerance_above = math.pi / 4.0
        joint_constraint.tolerance_below = 0.0
        joint_constraint.weight = 0.4
        constraints.joint_constraints.append(joint_constraint)
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
        orientation_constraint.link_name = 'left_RGB_camera'
        orientation_constraint.orientation.x = math.sqrt(0.5)
        orientation_constraint.orientation.y = 0.0
        orientation_constraint.orientation.z = -math.sqrt(0.5)
        orientation_constraint.orientation.w = 0.0
        
        orientation_constraint.absolute_x_axis_tolerance = math.pi / 4.0
        orientation_constraint.absolute_y_axis_tolerance = math.pi / 4.0
        orientation_constraint.absolute_z_axis_tolerance = math.pi / 10.0
        orientation_constraint.weight = 0.6
        constraints.orientation_constraints.append(orientation_constraint)

#         visibility_constraint = VisibilityConstraint()
#         visibility_constraint.target_radius = 0.25
#         visibility_constraint.cone_sides = 8
#         
#         camera_pose = PoseStamped()
#         camera_pose.header = Header(stamp = rospy.Time.now(), frame_id = 'left_RGB_camera')
#         camera_pose.pose.position.x = 0.0
#         camera_pose.pose.position.y = 0.0
#         camera_pose.pose.position.z = 0.03 # just to make this pose out of collision
#         camera_pose.pose.orientation.x = 0.0
#         camera_pose.pose.orientation.y = 0.0
#         camera_pose.pose.orientation.z = 0.0
#         camera_pose.pose.orientation.w = 1.0        
#         visibility_constraint.sensor_pose = camera_pose
# 
#         bin_center_pose = PoseStamped()
#         bin_center_pose.header = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
#         bin_center_pose.pose = g_bin_center_pose
#         visibility_constraint.target_pose = bin_center_pose
# 
#         visibility_constraint.max_view_angle = 0.0
#         visibility_constraint.max_range_angle = math.pi / 8.0
#         visibility_constraint.sensor_view_direction = 0 # SENSOR_Z
#         visibility_constraint.weight = 1.0
#         constraints.visibility_constraints.append(visibility_constraint)
    else:
        print "########### Error: Unkown constraints set with code: ", constraints_set

    return constraints


def get_limb_position_via_inverse_kinematics_motoman(robot_arm, endpoint_desired_state, ik_seed, constraints_set):
    global g_base_reference_frame
    global g_joints_state

    iksvc = rospy.ServiceProxy("compute_ik", GetPositionIK)
    ikreq = GetPositionIKRequest()
    if robot_arm == 0: # left
        ikreq.ik_request.group_name = "arm_left"
        ikreq.ik_request.ik_link_name = "head_sponge" #"left_RGB_camera" #"arm_left_link_7_t"
    else:
        ikreq.ik_request.group_name = "arm_right"
    ikreq.ik_request.robot_state = RobotState()
    hdr = Header(stamp = rospy.Time.now(), frame_id = g_base_reference_frame)
    ikreq.ik_request.robot_state.joint_state.header = hdr
    ikreq.ik_request.robot_state.joint_state.name = g_joints_state.keys()
    ikreq.ik_request.robot_state.joint_state.position = g_joints_state.values()
    ikreq.ik_request.avoid_collisions = True
    ikreq.ik_request.constraints = get_end_effector_constraint(constraints_set)
    ikreq.ik_request.pose_stamped = PoseStamped()
    ikreq.ik_request.pose_stamped.header.frame_id = g_base_reference_frame
    ikreq.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
    ikreq.ik_request.pose_stamped.pose = endpoint_desired_state
    ikreq.ik_request.timeout = rospy.Duration(3.0)
    try:
        rospy.wait_for_service("compute_ik", 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("######### Service call failed: %s" % (e,))
        sys.exit(0)
    if resp.error_code.val == resp.error_code.SUCCESS:
        # print resp
        limb_joints = get_joints_positions_from_joint_state(resp.solution.joint_state)
        # print limb_joints
    else:
        print("Could not find IK for given pose in get_limb_position_via_inverse_kinematics()")
        limb_joints = False
    
    return limb_joints


def get_limb_position_via_inverse_kinematics(robot_arm, endpoint_desired_state, ik_seed, constraints_set):
    global g_robot

    if g_robot == BAXTER:
        return get_limb_position_via_inverse_kinematics_baxter(robot_arm, endpoint_desired_state, ik_seed)
    else:
        return get_limb_position_via_inverse_kinematics_motoman(robot_arm, endpoint_desired_state, ik_seed, constraints_set)


def set_speed(plan, spd):
    n_joints = len(plan.joint_trajectory.joint_names)
    n_points = len(plan.joint_trajectory.points)
    
    new_plan = RobotTrajectory()
    # new_plan.joint_trajectory = JointTrajectory()
    new_plan.joint_trajectory.header = copy.deepcopy(plan.joint_trajectory.header)
    new_plan.joint_trajectory.joint_names = copy.deepcopy(plan.joint_trajectory.joint_names)
    new_plan.joint_trajectory.points = []
    for i in range(n_points):
        jtp = JointTrajectoryPoint()
        new_plan.joint_trajectory.points.append(jtp) 
        new_plan.joint_trajectory.points[i].time_from_start = plan.joint_trajectory.points[i].time_from_start / spd
        positions = []
        velocities = []
        accelerations = []
        for j in range(n_joints):
            positions.append(plan.joint_trajectory.points[i].positions[j])
            velocities.append(plan.joint_trajectory.points[i].velocities[j] * spd)
            accelerations.append(plan.joint_trajectory.points[i].accelerations[j] * spd)
        new_plan.joint_trajectory.points[i].positions = list(positions)
        new_plan.joint_trajectory.points[i].velocities = list(velocities)
        new_plan.joint_trajectory.points[i].accelerations = list(accelerations)

    return new_plan


def split_joints_configuration(desired_joints_configuration):
    global g_move_group
    global g_joints_state

    #print desired_joints_configuration
    #print g_joints_state
    new_joints_configuration = copy.deepcopy(desired_joints_configuration)
    for joint in desired_joints_configuration:
        new_joints_configuration[joint] = (desired_joints_configuration[joint] + g_joints_state[joint]) / 2.0

    return new_joints_configuration

    
def remove_joint_from_plan(plan, joint_name):
    if plan.joint_trajectory.joint_names.count(joint_name) != 0:
        index_of_the_joint = plan.joint_trajectory.joint_names.index(joint_name)
        plan.joint_trajectory.joint_names.pop(index_of_the_joint)
        n_points = len(plan.joint_trajectory.points)
        for i in range(n_points):
            positions = list(plan.joint_trajectory.points[i].positions)
            positions.pop(index_of_the_joint)
            plan.joint_trajectory.points[i].positions = positions
            velocities = list(plan.joint_trajectory.points[i].velocities)
            velocities.pop(index_of_the_joint)
            plan.joint_trajectory.points[i].velocities = velocities
            accelerations = list(plan.joint_trajectory.points[i].accelerations)
            accelerations.pop(index_of_the_joint)
            plan.joint_trajectory.points[i].accelerations = accelerations

    return


def add_missing_state(plan, joints_state):
    global g_robot

    if g_robot == BAXTER:
        return
    
    plan.joint_trajectory.header = Header(stamp = rospy.Time.now(), frame_id = '')
    joints_names = joints_state.keys()
    # remove_joint_from_plan(plan, 'head_hinge')
    s = set(plan.joint_trajectory.joint_names)
    missing_joints = [x for x in joints_names if x not in s]
    plan.joint_trajectory.joint_names.extend(missing_joints)
    missing_positions = []
    missing_velocities = []
    missing_accelerations = []
    for key in missing_joints:
        missing_positions.append(joints_state[key])
        missing_velocities.append(0.0)
        missing_accelerations.append(0.0)
    
    torso_joint_b1_pos = plan.joint_trajectory.joint_names.index('torso_joint_b1')
    torso_joint_b2_pos = plan.joint_trajectory.joint_names.index('torso_joint_b2')
    n_points = len(plan.joint_trajectory.points)
    for i in range(n_points):
        positions = list(plan.joint_trajectory.points[i].positions) + missing_positions
        plan.joint_trajectory.points[i].positions = positions
        velocities = list(plan.joint_trajectory.points[i].velocities) + missing_velocities
        plan.joint_trajectory.points[i].velocities = velocities
        accelerations = list(plan.joint_trajectory.points[i].accelerations) + missing_accelerations
        plan.joint_trajectory.points[i].accelerations = accelerations
        
        plan.joint_trajectory.points[i].positions[torso_joint_b2_pos] = plan.joint_trajectory.points[i].positions[torso_joint_b1_pos]
        plan.joint_trajectory.points[i].velocities[torso_joint_b2_pos] = plan.joint_trajectory.points[i].velocities[torso_joint_b1_pos]
        plan.joint_trajectory.points[i].accelerations[torso_joint_b2_pos] = plan.joint_trajectory.points[i].accelerations[torso_joint_b1_pos]

    return


def add_missing_state2(plan, joints_state):
    global g_robot

    if g_robot == BAXTER:
        return
    
    plan.joint_trajectory.header = Header(stamp = rospy.Time.now(), frame_id = '')
    joints_names = joints_state.keys()
    # remove_joint_from_plan(plan, 'head_hinge')
    s = set(plan.joint_trajectory.joint_names)
    missing_joints = [x for x in joints_names if x not in s]
    plan.joint_trajectory.joint_names.extend(missing_joints)
    missing_positions = []
    missing_velocities = []
    missing_accelerations = []
    for key in missing_joints:
        missing_positions.append(joints_state[key])
        missing_velocities.append(0.0)
        missing_accelerations.append(0.0)
    
    torso_joint_b1_pos = plan.joint_trajectory.joint_names.index('torso_joint_b1')
    torso_joint_b2_pos = plan.joint_trajectory.joint_names.index('torso_joint_b2')
    n_points = len(plan.joint_trajectory.points)
    new_plan = RobotTrajectory()
    new_plan.joint_trajectory.header = plan.joint_trajectory.header
    new_plan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names
    for i in [0]:
        positions = list(plan.joint_trajectory.points[i].positions) + missing_positions
        plan.joint_trajectory.points[i].positions = positions
        velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        plan.joint_trajectory.points[i].velocities = velocities
        accelerations = []
        plan.joint_trajectory.points[i].accelerations = accelerations
        
        plan.joint_trajectory.points[i].positions[torso_joint_b2_pos] = plan.joint_trajectory.points[i].positions[torso_joint_b1_pos]
#         plan.joint_trajectory.points[i].velocities[torso_joint_b2_pos] = plan.joint_trajectory.points[i].velocities[torso_joint_b1_pos]
#         plan.joint_trajectory.points[i].accelerations[torso_joint_b2_pos] = plan.joint_trajectory.points[i].accelerations[torso_joint_b1_pos]
    
    point = copy.deepcopy(plan.joint_trajectory.points[0])
    new_plan.joint_trajectory.points.append(point)
    point2 = copy.deepcopy(plan.joint_trajectory.points[0])
    new_plan.joint_trajectory.points.append(point2)
    new_plan.joint_trajectory.points[1].time_from_start.secs = 0
    new_plan.joint_trajectory.points[1].time_from_start.nsecs = 20000000
    
    return new_plan
    
    
def plan_and_move_to_joints_configuration_recursive(jc, speed, level):
    global g_move_group
    global g_joints_state

    print "plan_and_move_to_joints_configuration_recursive()"
    joints_configuration = copy.deepcopy(jc)
    tries = 0
    while (level > 0) and (tries <= 1):
        print "=== level ", level, "   tries ", tries 
        tries += 1
        g_move_group.clear_pose_targets()
        g_move_group.set_start_state_to_current_state()
        g_move_group.set_joint_value_target(joints_configuration)
        plan = g_move_group.plan()
        plan = set_speed(plan, speed)
        if not plan.joint_trajectory.points:
            print "Failed to plan go to target joints configuration recursive at level ", level
            new_joints_configuration = split_joints_configuration(joints_configuration)
            plan_and_move_to_joints_configuration_recursive(new_joints_configuration, speed, level - 1)
            continue
        print "We have a plan to go to a specific joints configuration!"
        add_missing_state(plan, g_joints_state)
        move_ok = g_move_group.execute(plan)
        print "@@@@@@@@@@@@@@@@@@@ move_ok"
        print move_ok
        if move_ok:
            print "Joints configuration achieved OKKKKK!!!"
            return True
        else:
            print "Could not execute the plan..."
        
    return False


def plan_and_move_to_the_current_position():
    global g_move_group
    global g_joints_state

    # The function is used for initializing the robot if it is not already initialized with a plan to the same place where it is.
    g_move_group.clear_pose_targets()
    g_move_group.set_start_state_to_current_state()
    g_move_group.set_joint_value_target(g_joints_state)
    plan = g_move_group.plan()
    if not plan.joint_trajectory.points:
        print "====================== No initialization plan..."
    else:
        plan = add_missing_state2(plan, g_joints_state)
        move_ok = g_move_group.execute(plan)
        if move_ok == False:
            print "====================== Could not go to the initialization plan..."
#         rospy.sleep(10000000)

    return            


def compare(joints_configuration, cjc, point):
    equal_conf = True
    for key in joints_configuration.keys():
        if joints_configuration[key] != cjc[key]:
            print key
            print joints_configuration
            print cjc
            equal_conf = False
            break
    if equal_conf:
        return ""
    else:
        return "@@@@@@@@@@@@@%%%%%^^^^^^^^ DIFFERENT in point ", point
    

def plan_and_move_to_joints_configuration(jc, speed):
    global g_move_group
    global g_joints_state
    global g_first_plan_and_move

    print "plan_and_move_to_joints_configuration()"
    joints_configuration = copy.deepcopy(jc)
    cjc = copy.deepcopy(jc)
    for tries in range(1, 4):
        print "try planning no. ", tries
        if g_first_plan_and_move: # initialize Motoman servos
            print "+++++++++ Planning and moving to the same place for robot's servos initialization..."
            plan_and_move_to_the_current_position()
            g_first_plan_and_move = False
        g_move_group.clear_pose_targets()
        g_move_group.set_start_state_to_current_state()
        g_move_group.set_joint_value_target(joints_configuration)
        print compare(joints_configuration, cjc, 0),
        plan = g_move_group.plan()
        print compare(joints_configuration, cjc, 1),
        plan = set_speed(plan, speed)
        print compare(joints_configuration, cjc, 2),
        if not plan.joint_trajectory.points:
            if tries == 3:
                print "@@@@@@@@@@@@@ Failed 3 times..."
                return False # No recursive!
                move_ok = plan_and_move_to_joints_configuration_recursive(joints_configuration, speed, 3)
                if move_ok:
                    return plan_and_move_to_joints_configuration(joints_configuration, speed) # to guarantee we reached the target configuration
                else:
                    return False
            else:
                print "Failed to plan to go to target joints configuration no. ", tries
                continue
        print "We have a plan to go to a specific joints configuration!"
        print compare(joints_configuration, cjc, 3),
        add_missing_state(plan, g_joints_state)
        print compare(joints_configuration, cjc, 4),
        move_ok = g_move_group.execute(plan)
        print compare(joints_configuration, cjc, 5),
        if move_ok:
            print "Joints configuration achieved OKKKKK!!!"
            return True
        else:
            print "Could not execute the plan..."
    
    return False


def plan_and_move_end_effector_to(robot_arm, end_effector_pose, speed, ik_seed, constraints_set):
    global g_move_group

    local_end_effector_pose = copy.deepcopy(end_effector_pose)
    print "plan_and_move_end_effector_to(end_effector_pose)"
    print "has end effector according to moveit?", g_move_group.has_end_effector_link()
    print "end effector name(s) = ", g_move_group.get_end_effector_link()
    # print end_effector_pose
    for tries in range(1, 6):
        limb_joints = get_limb_position_via_inverse_kinematics(robot_arm, local_end_effector_pose, ik_seed, constraints_set)
        if limb_joints == False:
            if tries == 5:
                print "@@@@@@@@@@@@@ Failed 5 times..."
                return False
            else:
                print "Failed to find IK to go to target end effector pose no. ", tries
                local_end_effector_pose.position.z += 0.01 * (random.random() - 0.5)
                continue
        else:
            break
    #print limb_joints
    
    return plan_and_move_to_joints_configuration(limb_joints, speed)


def add_box_to_scene(box_name, box_pose, box_dimensions):
    global g_scene

    g_scene.add_box(box_name, box_pose, box_dimensions)

    return


def add_mesh_to_scene(mesh_name, mesh_pose, mesh_file_name):
    global g_scene
    
    g_scene.add_mesh(mesh_name, mesh_pose, mesh_file_name)

    return


def remove_object_from_scene(object_name, header):
    global g_scene
    
    g_scene.remove_world_object2(object_name, header)
    
    return


def attach_mesh_to_robot(robot_arm, robot_link, mesh_name, pose_in_object, mesh_file_name):
    global g_scene

    if g_robot == BAXTER:
        if robot_arm == 0: # left
            g_scene.attach_mesh(robot_link, mesh_name, pose_in_object, mesh_file_name, (1,1,1), ['left_gripper_base', 'left_gripper'])
        else:
            g_scene.attach_mesh(robot_link, mesh_name, pose_in_object, mesh_file_name, (1,1,1), ['right_gripper_base', 'right_gripper'])
    else:
        if robot_arm == 0: # left
            g_scene.attach_mesh(robot_link, mesh_name, pose_in_object, mesh_file_name, (1,1,1), ['head_sponge', 'head_base'])
        else:
            g_scene.attach_mesh(robot_link, mesh_name, pose_in_object, mesh_file_name, (1,1,1), ['arm_right_link_7_t', 'arm_right_link_tool0'])
    
    return


def remove_mesh_from_robot(link, mesh_name):
    global g_scene

    g_scene.remove_attached_object(link, mesh_name)
    
    return


def clear_octomap():
#    global g_scene_publisher

    print "== Clear octomap"
    rospy.sleep(1.0)
    octomap_eraser_pose = PoseStamped()
    octomap_eraser_pose.header = Header(stamp = rospy.Time.now(), frame_id = '/world')
    octomap_eraser_pose.pose.position.x = 0.0
    octomap_eraser_pose.pose.position.y = 0.0
    octomap_eraser_pose.pose.position.z = 1.5
    octomap_eraser_pose.pose.orientation.x = 0.0
    octomap_eraser_pose.pose.orientation.y = 0.0
    octomap_eraser_pose.pose.orientation.z = 0.0
    octomap_eraser_pose.pose.orientation.w = 1.0
    add_box_to_scene("octomap_eraser", octomap_eraser_pose, (0.05, 0.05, 0.05)) # delete the previous octomap
    rospy.sleep(1.0)
    
    return
    
#     rospy.wait_for_service('/get_planning_scene', 5.0)
#     get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
#     request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
#     response = get_planning_scene(request)
#     acm = response.scene.allowed_collision_matrix
#     
#     # We just publish a scene diff without any difference. 
#     # A hack in planning_scene_monitor.cpp clear the octomap if the octomap_eraser cuboid is added to or removed from the scene.
#     planning_scene_diff = PlanningScene(is_diff=True)
#     g_scene_publisher.publish(planning_scene_diff)
#     
#     return


def update_planning_library_state(joints_state):
    global g_joints_state
    global g_joints_state_available
    
    g_joints_state = joints_state
    g_joints_state_available = True

    return

def read_command_line():
    # THIS SHOULD BE IN THE init_planing_library FOR MOVEIT
    moveit_commander.roscpp_initialize(sys.argv)
    
    return


def set_move_group(move_group_name):
    global g_move_group

    g_move_group = moveit_commander.MoveGroupCommander(move_group_name)
    # g_move_group.set_planner_id("RRTConnectkConfigDefault")
    # g_move_group.set_planner_id("RRTstarkConfigDefault")
    rospy.sleep(0.5)
   
    return


def init_planning_library(move_group_name, robot):
    global g_scene
    global g_move_group
    global g_scene_publisher
    global g_joints_state_available
    global g_base_reference_frame
    global g_robot
    global g_first_plan_and_move

    g_robot = robot
    if g_robot == BAXTER:
        g_base_reference_frame = "/base"
    else: # Motoman
        g_base_reference_frame = "/base_link"

    # MoveIt!
    g_first_plan_and_move = True
    g_scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    set_move_group(move_group_name)
    # g_scene_publisher = rospy.Publisher('planning_scene', PlanningScene, queue_size = 1)
    
    g_joints_state_available = False

    return
