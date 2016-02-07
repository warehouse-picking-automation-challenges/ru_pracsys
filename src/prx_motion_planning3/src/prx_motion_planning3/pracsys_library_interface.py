#!/usr/bin/env python

import rospy
import copy

# from baxter_core_msgs.srv import (
#     SolvePositionIK,
#     SolvePositionIKRequest,
# )

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

from prx_planning.srv import *

import random


def get_limb_postion_via_inverse_kinematics(endpoint_desired_state, ik_seed):
    # global g_origin_robot_position # TODO: This should be the current arm configuration...
    
    global g_arm_choice
    ns = "validate_end_effector"
    iksvc = rospy.ServiceProxy(ns, validate_end_effector)
    ikreq = validate_end_effectorRequest()
    hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')
    
    pose = PoseStamped(hdr, endpoint_desired_state)
    ikreq.pose = pose
    ikreq.arm = g_arm_choice
    # js = JointState()
    # js.header = hdr
    # js.name = g_origin_robot_position.keys()
    # js.position = g_origin_robot_position.values()
    # ikreq.arm_config = js
    # print ikreq
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("######### Service call failed: %s" % (e,))
        sys.exit(0)
    if (resp.success):
        limb_joints = dict(zip(resp.arm_config.name, resp.arm_config.position))
    else:
        print("Could not find IK for given pose in get_limb_postion_via_inverse_kinematics()")
        limb_joints = False
    
    return limb_joints

def set_speed(plan, spd):
    return plan
    # new_plan = copy.deepcopy(plan)
    # n_joints = len(plan.joint_trajectory.joint_names)
    # n_points = len(plan.joint_trajectory.points)
        
    # for i in range(n_points):
    #     new_plan.joint_trajectory.points[i].time_from_start = plan.joint_trajectory.points[i].time_from_start / spd
    #     velocities = []
    #     accelerations = []
    #     for j in range(n_joints):
    #         velocities.append(plan.joint_trajectory.points[i].velocities[j] * spd)
    #         accelerations.append(plan.joint_trajectory.points[i].accelerations[j] * spd)
    #     new_plan.joint_trajectory.points[i].velocities = list(velocities)
    #     new_plan.joint_trajectory.points[i].accelerations = list(accelerations)

    # return new_plan


def plan_and_move_to_joints_configuration_recursive(joints_configuration, speed, level):
    plan_and_move_to_joints_configuration(joints_configuration,speed)


def plan_and_move_to_joints_configuration(joints_configuration, speed):
    # global g_move_group
    global g_arm_choice


    end_state = JointState();
    end_state.name.append(g_arm_choice+"_s0");
    end_state.name.append(g_arm_choice+"_s1");
    end_state.name.append(g_arm_choice+"_e0");
    end_state.name.append(g_arm_choice+"_e1");
    end_state.name.append(g_arm_choice+"_w0");
    end_state.name.append(g_arm_choice+"_w1");
    end_state.name.append(g_arm_choice+"_w2");

    end_state.position.append(joints_configuration[end_state.name[0]]);
    end_state.position.append(joints_configuration[end_state.name[1]]);
    end_state.position.append(joints_configuration[end_state.name[2]]);
    end_state.position.append(joints_configuration[end_state.name[3]]);
    end_state.position.append(joints_configuration[end_state.name[4]]);
    end_state.position.append(joints_configuration[end_state.name[5]]);
    end_state.position.append(joints_configuration[end_state.name[6]]);

    rospy.wait_for_service('compute_plan')
    rospy.wait_for_service('execute_plan')
    req = compute_planRequest()
    req.end_state = end_state
    req.arm = g_arm_choice

    print "plan_and_move_to_joints_configuration()"
    
    try:
        for tries in range(1, 10):
            query_motion_planner = rospy.ServiceProxy('compute_plan', compute_plan)
            execute_motion = rospy.ServiceProxy('execute_plan', execute_plan)
            plan = query_motion_planner(req);
            # # g_move_group.clear_pose_targets()
            # # g_move_group.set_start_state_to_current_state()
            # # g_move_group.set_joint_value_target(joints_configuration)
            # plan = g_move_group.plan()
            if not plan.success:
                if tries == 9:
                    print "@@@@@@@@@@@@@ Failed 9 times..."
                    return False
                else:
                    print "Failed to go to target joints configuration no. ", tries
                    
                    continue
            print "We have a plan to go to a specific joints configuration!"
            g = execute_motion();
            if g.success:
                print "Joints configuration achieved OKKKKK!!!"
                return True
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    return False



def plan_and_move_end_effector_to(arm_state,end_effector_pose,speed, ik_seed):
    print "plan_and_move_to_end_effector_pose() calling plan_and_move_to_joints_configuration(). end_effector_pose:"
    print end_effector_pose
    for tries in range(1, 10):
        limb_joints = get_limb_postion_via_inverse_kinematics(end_effector_pose,ik_seed)
        sleep(10)
        if limb_joints == False:
            if tries == 9:
                print "@@@@@@@@@@@@@ Failed 9 times..."
                return False
            else:
                print "Failed to plan to go to target end effector pose no. ", tries
                continue
        else:
            break
    return plan_and_move_to_joints_configuration(limb_joints,speed)


def add_box_to_scene(box_name, box_pose, box_dimensions):
    # global g_scene
    rospy.wait_for_service('place_object')
    req = place_objectRequest();
    req.object_pose = box_pose;
    req.object_name = box_name;
    print box_name
    try:
        place_object_caller = rospy.ServiceProxy('place_object', place_object)
        resp1 = place_object_caller(req);
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return


def add_mesh_to_scene(mesh_name, mesh_pose, mesh_file_name):
    rospy.wait_for_service('place_object')
    req = place_objectRequest();
    req.object_pose = mesh_pose;
    req.object_name = mesh_name;
    print mesh_name
    try:
        place_object_caller = rospy.ServiceProxy('place_object', place_object)
        resp1 = place_object_caller(req);
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return


def remove_object_from_scene(object_name, header):
    if "_attached" in object_name:
        return
    if object_name == "":
        return

    print object_name
    print "\n\n\n\n\n\n\n\n"
    rospy.wait_for_service('clear_object')
    req = clear_objectRequest();
    req.object_name = object_name;
    try:
        clear_object_caller = rospy.ServiceProxy('clear_object', clear_object)
        resp1 = clear_object_caller(req);
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    return


def attach_mesh_to_robot(robot_arm, robot_link, mesh_name, pose_in_object, mesh_file_name):
    # global g_scene
    grasp_svc = rospy.ServiceProxy("grasp", grasp)
    mesh_name = mesh_name.replace("_attached","")
    try:
        rospy.wait_for_service("grasp", 5.0)
        resp = grasp_svc(mesh_name)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("######### Service call failed: %s" % (e,))
        sys.exit(0)    
    return


def remove_mesh_from_robot(object_name, mesh_name):
    release_svc = rospy.ServiceProxy("release", grasp)
    try:
        rospy.wait_for_service("release", 5.0)
        resp = release_svc(object_name)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("######### Service call failed: %s" % (e,))
        sys.exit(0)    
    return

def clear_octomap():
    rospy.wait_for_service('clear_object')
    try:
        clear_object_caller = rospy.ServiceProxy('clear_object', clear_object)
        resp1 = clear_object_caller("map");
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e  
    return

def update_planning_library_state(left_joints_state):
    global g_left_joints_state
    global g_left_joints_state_available
    
    g_left_joints_state = left_joints_state
    g_left_joints_state_available = True

    return

def split_joints_configuration(desired_joints_configuration):
    global g_left_joints_state

    #print desired_joints_configuration
    #print g_joints_state
    new_joints_configuration = copy.deepcopy(desired_joints_configuration)
    for joint in desired_joints_configuration:
        new_joints_configuration[joint] = (desired_joints_configuration[joint] + g_left_joints_state[joint]) / 2.0

    return new_joints_configuration

def read_command_line():
    # THIS SHOULD BE IN THE init_planing_library FOR MOVEIT
    pass

def init_planning_library(arm_name,blah):
    global g_arm_choice
    # global g_scene
    # global g_move_group
    # global g_scene_publisher
    # global g_origin_robot_position

    # # MoveIt!
    # g_scene = moveit_commander.PlanningSceneInterface()
    # robot = moveit_commander.RobotCommander()
    # g_move_group = moveit_commander.MoveGroupCommander("left_arm")
    # g_scene_publisher = rospy.Publisher('planning_scene', PlanningScene, queue_size = 10)
    
    g_left_joints_state_available = False
    if arm_name=="left_arm":
        g_arm_choice = "left"
    elif arm_name=="right_arm":
        g_arm_choice = "right"

    # g_origin_robot_position = {'left_w0': -2.3285828333496097, 'left_w1': 1.0131943092407227, 'left_w2': 1.523242920629883, 'left_e0': -0.9506845922058106, 'left_e1': 2.2495828228637698, 'left_s0': 0.6201117327941895, 'left_s1': 0.5553010445800781}

    

