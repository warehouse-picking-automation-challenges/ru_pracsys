#!/usr/bin/env python

##############################################################################################
# Imports                                                                                    #
##############################################################################################
    
import sys 
import rospy
import math
import numpy
import time
import copy
import tf
import random

import std_msgs.msg
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import JointState, Image

import actionlib
from actionlib_msgs.msg import *
from robotiq_force_torque_sensor.msg import *

# from time import sleep

from prx_decision_making.msg import DecisionMakingStateMessage,MotionPlanningStateMessage
from prx_planning.msg import compute_planAction, compute_planGoal, execute_planAction, execute_planGoal
from prx_planning.srv import *


##############################################################################################
# Definitions                                                                                #
##############################################################################################

SIMULATING = True

if SIMULATING:
    ROBOT_SPEED_SLOW = 0.4
    ROBOT_SPEED_FAST = 0.4
else:
    ROBOT_SPEED_SLOW = 0.5
    ROBOT_SPEED_FAST = 0.7


X_GRIPPING_OFFSET = 0.08
X_MAPPING_OFFSET = 0.48

FAST_DETECTION_MODE = False
ROBUST_POSE_TIMEOUT = 5

MAPPING_ANGLE = 0
MAPPING = False

DETECTION_TIMEOUT = 1

LEFT_ARM = 0
RIGHT_ARM = 1

CAMERA = 0
END_EFFECTOR = 1

HARD_CODED_OBJECT_POSITION = Pose()
HARD_CODED_OBJECT_POSITION.position.x = 1.0
HARD_CODED_OBJECT_POSITION.position.y = 0.27
HARD_CODED_OBJECT_POSITION.position.z = 0.95 
HARD_CODED_OBJECT_POSITION.orientation.x = 0#math.sqrt(.5)
HARD_CODED_OBJECT_POSITION.orientation.y = 0
HARD_CODED_OBJECT_POSITION.orientation.z = 0 
HARD_CODED_OBJECT_POSITION.orientation.w = 1#math.sqrt(.5)

GRASP_THRESHOLD=10
GRASP_ERROR=2

##############################################################################################
# Support Functions                                                                          #
##############################################################################################

from orientation_helpers import *

##############################################################################################
# Robotiq Gripper Class                                                                      #
##############################################################################################

from robotiq_gripper import *

##############################################################################################
# UniGripper Class                                                                           #
##############################################################################################

from unigripper import *


##############################################################################################
# Shelf and Order Bin Classes                                                                #
##############################################################################################

from apc_objects import *
    
##############################################################################################
# Task Planner Class                                                                         #
##############################################################################################

class TaskPlanner:

    ##### Initialization Functions #####
    def __init__(self):
        rospy.init_node('task_planner', anonymous=False)

        self.shelf = "shelf"
        self.change_shelf = False
        self.new_shelf = "shelf"
        # Initializations
        self.g_decision_making_state = None
        self.g_decision_making_state_available = False
        self.g_object_attached_to_the_gripper = False
        self.g_decision_making_previous_state = None
        self.g_base_reference_frame = "/base_link"

        self.tf_broadcaster = tf.TransformBroadcaster()

        if rospy.has_param('/hard_coded_object'):
            self.hard_coded_object = rospy.get_param("/hard_coded_object")
        else:
            self.hard_coded_object = False
        print "HARD_CODED_OBJECT: " + str(self.hard_coded_object)

        self.g_shelf_description = ShelfDescription('shelf_description_motoman.txt')
        print "Shelf description: " + str(self.g_shelf_description)

        self.g_order_bin_description = OrderBinDescription('order_bin_description_motoman.txt')
        print "Order bin descrition:" + str(self.g_order_bin_description)

        self.precompute_states()
        self.precompute_configs()
        	        
        # add the shelf and the order bin
        self.add_object_to_scene( "shelf", self.g_shelf_description )
        self.add_object_to_scene( "order_bin", self.g_order_bin_description )

        #Move arms to origin position
        print "=== Move arms to origin position: "  
        self.left_gripper = UniGripper()
        self.moving_arm = LEFT_ARM;
        if self.move_to_state( LEFT_ARM, self.states[LEFT_ARM]['origin_state'], ROBOT_SPEED_FAST,"origin") == False:
            sys.exit(-1)
        self.moving_arm = RIGHT_ARM;
        if self.move_to_state( RIGHT_ARM, self.states[RIGHT_ARM]['origin_state'], ROBOT_SPEED_FAST,"origin") == False:
            sys.exit(-1)     
        self.moving_arm = LEFT_ARM;          
        self.previous_moving_arm = LEFT_ARM

        print "=== Finished moving arms to origin position"

        # Subscribers 
        rospy.Subscriber("/decision_making_state", DecisionMakingStateMessage, self.update_decision_state_callback, queue_size=1)
        rospy.Subscriber("/ft_feedback", ft_feedback, self.update_ft_feedback, queue_size=1)

        # Publishers
        self.g_motion_planner_state_publisher = rospy.Publisher('/task_planner_state', MotionPlanningStateMessage, queue_size=1)

        # PREVIOUS VERSION WAS CALLING INITIALIZATION OF GRIPPER TWICE??
        rospy.Timer(rospy.Duration(0.25), self.task_planner)
        rospy.spin()

        return

    # ORIGIN_STATE and ORDER_STATE
    def precompute_states( self ):    
        self.states = {}

        self.states[LEFT_ARM] = {}
        self.states[LEFT_ARM]['origin_state'] = {'torso_joint_b1': 0, 'arm_left_joint_1_s': 1.57,'arm_left_joint_2_l': 0,'arm_left_joint_3_e': 0,'arm_left_joint_4_u': -1.70,'arm_left_joint_5_r': 0,'arm_left_joint_6_b': 0,'arm_left_joint_7_t': 0,'head_hinge':0}
        self.states[LEFT_ARM]['order_state'] = {'torso_joint_b1': -1.57, 'arm_left_joint_1_s': 1.6616592407226562,'arm_left_joint_2_l': 0.6775082349777222,'arm_left_joint_3_e': 0,'arm_left_joint_4_u': -1.1201856136322021,'arm_left_joint_5_r': 0.0,'arm_left_joint_6_b': -0.16566991806030273,'arm_left_joint_7_t': 0,'head_hinge':0}

        self.states[RIGHT_ARM] = {}
        self.states[RIGHT_ARM]['origin_state'] = {'torso_joint_b1': 0, 'arm_right_joint_1_s': 1.57,'arm_right_joint_2_l': 0,'arm_right_joint_3_e': 0,'arm_right_joint_4_u': -1.7,'arm_right_joint_5_r': 0,'arm_right_joint_6_b': 0,'arm_right_joint_7_t': 0}
        self.states[RIGHT_ARM]['order_state'] = {'torso_joint_b1': 1.57, 'arm_right_joint_1_s': 1.6616592407226562,'arm_right_joint_2_l': 0.6775082349777222,'arm_right_joint_3_e': 0,'arm_right_joint_4_u': -1.1201856136322021,'arm_right_joint_5_r': 0,'arm_right_joint_6_b': -0.16566991806030273,'arm_right_joint_7_t': 0}

    def precompute_configs(self):
        x_mapping = self.g_shelf_description.x - X_MAPPING_OFFSET
        self.configs = {}
        for arm in 0, 1: # 0 = left, 1 = right
            self.configs[arm] = {}
            for bin in 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L':
                self.configs[arm][bin] = {}
                pose = Pose()
                pose.position.x = self.g_shelf_description.x - X_GRIPPING_OFFSET
                pose.orientation = Quaternion(0.0,math.sqrt(0.5),0.0,math.sqrt(0.5))
                
                 # GRIPPER CONFIG
                y_center_bin, bin_width = self.g_shelf_description.get_y_bin_center(bin)
                z_center_bin, bin_height = self.g_shelf_description.get_z_bin_center(bin)

                pose_i = copy.deepcopy(pose)
                pose_i.position.y = y_center_bin
                pose_i.position.z = z_center_bin
                #RIGHT ARM
                if arm == 1 and bin in ['A', 'B', 'C', 'D',  'F', 'G', 'H', 'I']:
                    helpful_orientation = Quaternion( 0.0, 0.0, 1.0, 0.0 )
                    pose_i.orientation = rotate_quaternion( pose.orientation, helpful_orientation )
                #LEFT ARM
                if arm == 0 and bin in ['E','F','J','K','L']:
                    helpful_orientation = Quaternion( 0.0, 0.0, 1.0, 0.0 )
                    pose_i.orientation = rotate_quaternion( pose.orientation, helpful_orientation )
                self.configs[arm][bin]['gripper_config'] = pose_i

                # BACKUP ORDER BIN
                backup_pose = copy.deepcopy(pose)
                backup_pose.position.x = 0.4
                backup_pose.position.y = 0.0
                backup_pose.position.z = 0.3
                backup_pose.orientation = Quaternion( 0.0, 0.98480775301, 0.0, -0.17364817766 )
                self.configs[arm][bin]['backup_config'] = backup_pose

                # MAPPING CONFIG
                pose_j = copy.deepcopy(pose_i)
                pose_j.position.x = x_mapping
                pose_j.orientation = copy.deepcopy(pose.orientation)
                
                robotiq_heights = {'A': 0.04, 'B': 0.04, 'C': 0.055, 'D': 0.04, 'E': 0.04, 'F': 0.04, 'G': 0.04, 'H': 0.04, 'I': 0.04, 'J': 0.04, 'K': 0.04, 'L': 0.04}
                unigripper_heights = {'A': 0.04, 'B': 0.04, 'C': 0.04, 'D': 0.04, 'E': 0.04, 'F': 0.04, 'G': 0.04, 'H': 0.04, 'I': 0.04, 'J': 0.04, 'K': 0.04, 'L': 0.04}
                map_heights = [ unigripper_heights, robotiq_heights ]
                
                offset_height = map_heights[arm][bin]
                pose_j.position.z += offset_height
                pitch_axis = Point(0,1,0)
                pitch_angle = math.atan2(offset_height, 0.67)
                
                if bin in ['D','E','F','J','K','L']:
                    helpful_orientation = Quaternion( 0.0, 0.0, 1.0, 0.0)
                    pose_j.orientation = rotate_quaternion( pose.orientation, helpful_orientation )
                    pitch_angle *= -1.0
                # if arm == 0 and bin in ['F']:
                #     helpful_orientation = Quaternion( 0.0, 0.0, 1.0, 0.0)
                #     pose_j.orientation = rotate_quaternion( pose_j.orientation, helpful_orientation )
                #     pitch_angle *= -1.0

                pitch_q = axis_angle_to_quaternion( pitch_axis, pitch_angle )
                pose_j.orientation = rotate_quaternion( pose_j.orientation, pitch_q )

                self.configs[arm][bin]['map_config'] = pose_j

                # DETECT-1 CONFIG
                pose_k = copy.deepcopy(pose_j)
                axis = Point(1,0,0)
                rot_q = axis_angle_to_quaternion(axis,-(MAPPING_ANGLE*3.14159265/180.0))
                pose_k.position.y = pose_k.position.y - .1
                pose_k.orientation = rotate_quaternion(pose_k.orientation,rot_q)
                self.configs[arm][bin]['detect_config_1'] = pose_k

                # DETECT-2 CONFIG
                pose_l = copy.deepcopy(pose_j)
                rot_q = axis_angle_to_quaternion(axis,(MAPPING_ANGLE*3.14159265/180.0))
                pose_l.position.y = pose_l.position.y + .1
                pose_l.orientation = rotate_quaternion(pose_l.orientation,rot_q)
                self.configs[arm][bin]['detect_config_2'] = pose_l

                # HINT 0 CONFIG
                pose_h0 = copy.deepcopy(pose_i)
                pose_h0.position.x = self.g_shelf_description.x
                pose_h0.position.y += (bin_width / 2.0)
                pose_h0.position.z -= (bin_height / 2.0)
                pose_h0.orientation = copy.deepcopy(pose.orientation)
                self.configs[arm][bin]['pose_hint0'] = pose_h0

                # HINT 1 CONFIG
                pose_h1 = copy.deepcopy(pose_i)
                pose_h1.position.x = self.g_shelf_description.x
                pose_h1.position.y += (bin_width / 2.0)
                pose_h1.position.z += (bin_height / 2.0 )
                pose_h1.orientation = copy.deepcopy(pose.orientation)
                self.configs[arm][bin]['pose_hint1'] = pose_h1

                # HINT 2 CONFIG
                pose_h2 = copy.deepcopy(pose_i)
                pose_h2.position.x = self.g_shelf_description.x
                pose_h2.position.y -= (bin_width / 2.0)
                pose_h2.position.z += (bin_height / 2.0)
                pose_h2.orientation = copy.deepcopy(pose.orientation)
                self.configs[arm][bin]['pose_hint2'] = pose_h2

                # HINT 3 CONFIG
                pose_h3 = copy.deepcopy(pose_i)
                pose_h3.position.x = self.g_shelf_description.x
                pose_h3.position.y -= (bin_width / 2.0)
                pose_h3.position.z -= (bin_height / 2.0)
                pose_h3.orientation = copy.deepcopy(pose.orientation)
                self.configs[arm][bin]['pose_hint3'] = pose_h3

    def update_ft_feedback(self,data):
        #ignore for now
        print "Force-torque preemption: "+str(data.preempted)
        self.state = 13

    def update_decision_state_callback(self,data):
        self.g_decision_making_state = data
        self.g_decision_making_state.bin_id = chr(self.g_decision_making_state.bin_id)
        self.latest_moving_arm = self.g_decision_making_state.robot_arm

        if SIMULATING == False and MAPPING:
            if "bin_"+str(self.g_decision_making_state.bin_id) != self.shelf:
                self.change_shelf = True
                self.new_shelf = "bin_"+str(self.g_decision_making_state.bin_id)
                self.remove_object_from_scene(self.shelf)
                self.shelf = "bin_"+str(self.g_decision_making_state.bin_id);
                self.add_object_to_scene( self.shelf, self.g_shelf_description )

        if self.g_decision_making_previous_state != None:
            if self.g_decision_making_state.state != self.g_decision_making_previous_state.state:
                self.g_decision_making_state_available = True
        else:
            self.g_decision_making_state_available = True
        self.g_decision_making_previous_state = self.g_decision_making_state

        # Publish objec_pose_hitX tf transforms
        bin_id = self.g_decision_making_state.bin_id
        orient = quat_from_orient(self.configs[self.moving_arm][bin_id]['pose_hint0'].orientation)
        pos = position_as_list(self.configs[self.moving_arm][bin_id]['pose_hint0'].position)
        self.tf_broadcaster.sendTransform( pos, orient, rospy.Time.now(), "/object_pose_hint0", self.g_base_reference_frame )

        pos = position_as_list(self.configs[self.moving_arm][bin_id]['pose_hint1'].position)
        self.tf_broadcaster.sendTransform( pos, orient, rospy.Time.now(), "/object_pose_hint1", self.g_base_reference_frame)

        pos = position_as_list(self.configs[self.moving_arm][bin_id]['pose_hint2'].position)
        self.tf_broadcaster.sendTransform( pos, orient, rospy.Time.now(), "/object_pose_hint2", self.g_base_reference_frame)

        pos = position_as_list(self.configs[self.moving_arm][bin_id]['pose_hint3'].position)
        self.tf_broadcaster.sendTransform( pos, orient, rospy.Time.now(), "/object_pose_hint3", self.g_base_reference_frame)

        return

    def add_object_to_scene(self,name,description):
        print "== Adding object to the scene: " + name
        rospy.wait_for_service('place_object')
        req = place_objectRequest();
        req.object_name = name;

        req.object_pose = PoseStamped()
        req.object_pose.header = Header(stamp = rospy.Time.now(), frame_id = '/world')
        req.object_pose.pose.position.x = description.x + description.depth / 2.0
        req.object_pose.pose.position.y = description.y
        req.object_pose.pose.position.z = description.z + description.height / 2.0
        radtodeg = math.pi / 180.0
        obj_or = tf.transformations.quaternion_from_euler(
            description.roll * radtodeg, description.pitch * radtodeg, description.yaw * radtodeg)
        req.object_pose.pose.orientation = Quaternion(obj_or[0],obj_or[1],obj_or[2],obj_or[3])

        try:
            place_object_caller = rospy.ServiceProxy('place_object', place_object)
            resp1 = place_object_caller(req);
        except rospy.ServiceException, e:
            print "Service call failed: "

    def move_to_state(self, the_arm, the_state, speed, origin_or_order_bin ):
        end_state = JointState();
        if the_arm == LEFT_ARM:
            end_state.name.append("arm_left_joint_1_s");
            end_state.name.append("arm_left_joint_2_l");
            end_state.name.append("arm_left_joint_3_e");
            end_state.name.append("arm_left_joint_4_u");
            end_state.name.append("arm_left_joint_5_r");
            end_state.name.append("arm_left_joint_6_b");
            end_state.name.append("arm_left_joint_7_t");
            end_state.name.append("torso_joint_b1");
            end_state.name.append("head_hinge");
        else:
            end_state.name.append("arm_right_joint_1_s");
            end_state.name.append("arm_right_joint_2_l");
            end_state.name.append("arm_right_joint_3_e");
            end_state.name.append("arm_right_joint_4_u");
            end_state.name.append("arm_right_joint_5_r");
            end_state.name.append("arm_right_joint_6_b");
            end_state.name.append("arm_right_joint_7_t");
            end_state.name.append("torso_joint_b1");

        end_state.position.append(the_state[end_state.name[0]]);
        end_state.position.append(the_state[end_state.name[1]]);
        end_state.position.append(the_state[end_state.name[2]]);
        end_state.position.append(the_state[end_state.name[3]]);
        end_state.position.append(the_state[end_state.name[4]]);
        end_state.position.append(the_state[end_state.name[5]]);
        end_state.position.append(the_state[end_state.name[6]]);
        end_state.position.append(the_state[end_state.name[7]]);
        if the_arm == LEFT_ARM:
            end_state.position.append(the_state[end_state.name[8]]);

        cclient = actionlib.SimpleActionClient('compute_plan', compute_planAction)
        cclient.wait_for_server()

        cgoal = compute_planGoal()
        cgoal.end_state = end_state
        if the_arm == LEFT_ARM:
            cgoal.arm = "left"
        else:
            cgoal.arm = "right"
        cgoal.dest_type = origin_or_order_bin
        cclient.send_goal(cgoal)
        cclient.wait_for_result()

        if cclient.get_state() != GoalStatus.SUCCEEDED:
            print "Failed to plan to target configuration"
            return False

        print "We have a plan to go to a specific configuration!"
        return self.execute_plan(speed);

    def execute_plan(self,speed,traj_type="query"):

        eclient = actionlib.SimpleActionClient('execute_plan', execute_planAction)
        eclient.wait_for_server()
        egoal = execute_planGoal()
        egoal.speed = speed
        egoal.traj_type = traj_type
        eclient.send_goal(egoal)
        eclient.wait_for_result()
        if eclient.get_state() != GoalStatus.SUCCEEDED:
            print "Failed to execute movement to target configuration"
            return False

        print "Target state achieved!"
        return True

    ##### Main Task Planner Operation #####
    def task_planner(self,timer_val):
        self.state = 0 # initially, no state
        if self.g_decision_making_state_available == True:
            self.g_decision_making_state_available = False
            current_motion_planning_mode = self.g_decision_making_state.motion_planning_mode

            if current_motion_planning_mode == 6: # mp_to_O -> move to origin
                print "============== Moving to origin =============="        

                rate = rospy.Rate(60) # hz
                counter = 0
                while counter < 300:
                    rospy.is_shutdown()
                    rate.sleep()
                    counter = counter+1

                self.moving_arm = self.latest_moving_arm
                       
                print "Current arm: " + str(self.moving_arm)
                # if self.moving_arm == RIGHT_ARM:
                #     self.right_gripper.close();
                # else:
                #     self.left_gripper.turn_off();
            
                print "Current bin: " + str(self.g_decision_making_state.bin_id)
                    
                if self.move_to_state( self.previous_moving_arm, self.states[self.previous_moving_arm]['origin_state'], ROBOT_SPEED_FAST, 'origin' ) == False:
                    print "Moving  arm " + str(self.previous_moving_arm) + " to origin - Failure"
                    self.state = 13 # COLLISION DETECTED
                else:
                    print "Moved arm " + str(self.previous_moving_arm) + " to origin - Success"                
                    self.state = 1 # robots are at the origin

                self.previous_moving_arm = self.moving_arm
                print "Current bin: " + str(self.g_decision_making_state.bin_id)

            if current_motion_planning_mode == 7: # mp_to_B -> move to Bin
                print "============== Moving to bin =============="
                if self.change_shelf:
                    self.remove_object_from_scene(self.shelf)
                    self.shelf = self.new_shelf
                    self.change_shelf = False
                    self.add_object_to_scene( self.new_shelf, self.g_shelf_description )

                if FAST_DETECTION_MODE:
                    self.state = 2
                else:
                    if self.move_to_config( CAMERA, self.g_decision_making_state.bin_id, 'detect_config_1', ROBOT_SPEED_FAST ) == False:
                        print "Moving to configuration failed"
                        self.state = 13 # COLLISION DETECTED
                    else:
                        self.state = 2 # robot is at the bin

            if current_motion_planning_mode == 14: # mp_to_S -> Warm up SLAM
                print "============== Warm up SLAM =============="
                rospy.sleep(2)
                self.state = 9 # SLAM_WARMED_UP = 9
  
                if self.move_to_config( CAMERA, self.g_decision_making_state.bin_id, 'map_config', ROBOT_SPEED_FAST ) == False:
                    sys.exit(-1)    
                val = raw_input("Key enter to move the camera right side up ")

                helpful_orientation = Quaternion( 0.0, 0.0, 1.0, 0.0 )
                self.configs[self.moving_arm][self.g_decision_making_state.bin_id]['map_config'].orientation = rotate_quaternion( self.configs[self.moving_arm]['E']['map_config'].orientation, helpful_orientation ) 
                if self.move_to_config( CAMERA, self.g_decision_making_state.bin_id, 'map_config', ROBOT_SPEED_FAST ) == False:
                    sys.exit(-1)     
                sys.exit(-1)



            if current_motion_planning_mode == 15: # mp_to_D1 -> Move to Detect 1
                print "============== Move to Detect 1 =============="

                if FAST_DETECTION_MODE:
                    self.state = 10 # MOVE TO DETECT 1
                else: 
                    rospy.sleep(ROBUST_POSE_TIMEOUT)
                    if self.move_to_config( CAMERA, self.g_decision_making_state.bin_id, 'detect_config_2', ROBOT_SPEED_FAST ) == False:
                        print "Moving to configuration failed"
                        self.state = 13 # COLLISION DETECTED
                    else:
                        self.state = 10 # MOVED_TO_DETECT1 = 10

            if current_motion_planning_mode == 16: # mp_to_D2 -> Move to Detect 2
                print "============== Move to Detect 2 =============="
                rospy.sleep(ROBUST_POSE_TIMEOUT)
                if self.move_to_config( CAMERA, self.g_decision_making_state.bin_id, 'map_config', ROBOT_SPEED_FAST ) == False:
                    print "Moving to configuration failed"
                    self.state = 13 # COLLISION DETECTED
                else:
                    self.state = 11 # MOVED_TO_DETECT2 = 11

            if current_motion_planning_mode == 8: # mp_to_M -> move to map
                print "============== Mapping and Object Detection =============="
                self.map_and_detect()

            if current_motion_planning_mode == 9: # mp_to_Ob_ -> move closer to object
                print "============== Moving close to object =============="
                self.move_closer_to_object( self.g_decision_making_state.bin_id )

            if current_motion_planning_mode == 17: # mp_to_eval -> evaluate grasp
                print "============== Evaluate Grasp =============="
                self.evaluate_grasp()

            if current_motion_planning_mode == 12: # grasp -> grasp move
                print "============== Grasping =============="
                self.grasp( self.g_decision_making_state.bin_id)
    
            if current_motion_planning_mode == 10: # mp_to_AB_ -> move away from bin
                print "============== Moving away from bin =============="
                self.move_away_from_bin( self.g_decision_making_state.bin_id)

            if current_motion_planning_mode == 11: # mp_to_OB -> move to order bin
                print "============== Moving to order bin =============="
                self.clear_octomap()

                if self.move_to_state( self.moving_arm, self.states[self.moving_arm]['order_state'], ROBOT_SPEED_FAST, 'order_bin' ) == False:
                    print "Failed to go to the order bin the first time"
                    counter = 0
                    done_trying = False
                    temp_pose = copy.deepcopy( self.configs[self.moving_arm][self.g_decision_making_state.bin_id]['gripper_config'] )
                    while done_trying == False:
                        print "Failed to to go to the order bin -  Raising the target z coordinate and backing up locally"
                        counter += 1
                        if counter >= 5:
                            done_trying = True
                        else:
                            if self.move_to_config( END_EFFECTOR, self.g_decision_making_state.bin_id, 'backup_config', ROBOT_SPEED_FAST ):
                                done_trying = True
                            else:
                                temp_pose.position.x -= 0.04
                                self.move_to_general_config( END_EFFECTOR, temp_pose, ROBOT_SPEED_FAST )                        
                                self.configs[self.moving_arm][self.g_decision_making_state.bin_id]['backup_config'].position.z += 0.05

                    if counter >= 5:
                        self.state = 13
                    else:
                        self.state = 6
                    self.configs[self.moving_arm][self.g_decision_making_state.bin_id]['backup_config'].position.z = 0.3
                else:
                    self.state = 6

                if self.moving_arm == RIGHT_ARM:
                    print "Opening the Robotiq hand"
                    self.right_gripper.open("order_open")
                else:
                    print "Turning off the vacuum"
                    self.left_gripper.turn_off();
                self.dettach_object()

                if not self.grasp_success:
                    self.state = 13

            # this needs to happen in any case
            header = Header(stamp = rospy.Time.now(), frame_id = self.g_base_reference_frame)
            self.g_motion_planner_state_publisher.publish(header, self.state)
             
            print "=== motion planner new state = " + str(self.state) + "decision making state = " + str(self.g_decision_making_state.state) + "current motion planning mode = " + str(current_motion_planning_mode)

        print "@@@ === motion planner new state = " + str(self.state) + "decision making state = " + str(self.g_decision_making_state.state)

    def failsafe_origin( self ):
        if self.move_to_state( self.moving_arm, self.states[self.moving_arm]['origin_state'], ROBOT_SPEED_FAST, 'origin' ) == False:
            print "Failed to go to the origin the first time"
            counter = 0
            done_trying = False
            temp_pose = copy.deepcopy( self.configs[self.moving_arm][self.g_decision_making_state.bin_id]['gripper_config'] )
            while done_trying == False:
                print "Failed to to go to the order bin -  Backing up locally"
                counter += 1
                if counter >= 5:
                    done_trying = True
                else:
                    temp_pose.position.x -= 0.04
                    self.move_to_general_config( END_EFFECTOR, temp_pose, ROBOT_SPEED_FAST )
                    if self.move_to_state( self.moving_arm, self.states[self.moving_arm]['origin_state'], ROBOT_SPEED_FAST, 'origin' ) == True:
                        done_trying = True
                    
            if counter >= 5:
                return False
            else:
                return True
        else:
            return True

    def clear_octomap( self ):
        rospy.wait_for_service('clear_object')
        try:
            clear_object_caller = rospy.ServiceProxy('clear_object', clear_object)
            resp1 = clear_object_caller("map");
        except rospy.ServiceException, e:
            print "Service call failed"

    def move_to_config( self,link, the_bin, config_name, speed ):
        print "inside:: move_to_config() for link " + str(link) + " of robot " + str(self.moving_arm) + " at pose " + str(config_name) + " for bin " + str(the_bin)
        return self.move_to_general_config(link,self.configs[self.moving_arm][the_bin][config_name],speed)

    def move_to_general_config(self,link,config,speed):
        iksvc = rospy.ServiceProxy( "validate_end_effector", validate_end_effector)
        ikreq = validate_end_effectorRequest()
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')
        
        ikreq.pose = PoseStamped(hdr, config )
        if self.moving_arm == LEFT_ARM:
            ikreq.arm = "left"
        if self.moving_arm == RIGHT_ARM:
            ikreq.arm = "right"
        if link == CAMERA:
            ikreq.camera = True
        else:
            ikreq.camera = False
        try:
            rospy.wait_for_service( "validate_end_effector", 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("######### Service call failed: %s" % (e,))
            sys.exit(-1)
        if (resp.success):
            limb_joints = dict(zip(resp.arm_config.name, resp.arm_config.position))
            return self.execute_plan( speed, "IK" )
        else:
            print "Failure!"
            return False

    def map_and_detect( self ):        	    
        print "Get object pose"
        rospy.sleep(ROBUST_POSE_TIMEOUT)
        self.g_object_pose = False
        
        counter=0
        while self.g_object_pose == False and counter < DETECTION_TIMEOUT:
            self.g_object_pose = self.get_object_pose()
            counter = counter+1

        if self.g_object_pose == False:
            self.state = 12
            return False

        object_mesh_pose = PoseStamped()
        object_mesh_pose.header = Header(stamp = rospy.Time.now(), frame_id = '/world')
        object_mesh_pose.pose = self.g_object_pose
        object_mesh_pose.pose.orientation = rotate_orientation_quaternion(object_mesh_pose.pose.orientation)
        print "Adding the object mesh to the scene"  

        rospy.wait_for_service('place_object')
        req = place_objectRequest();
        req.object_pose = object_mesh_pose;
        req.object_name = self.g_decision_making_state.object_name;
        print self.g_decision_making_state.object_name;
        try:
            place_object_caller = rospy.ServiceProxy('place_object', place_object)
            resp1 = place_object_caller(req);
        except rospy.ServiceException, e:
            print "Service call failed:"            
        print "Object model added - Detection success"  
        #self.remove_object_from_scene("shelf")

        self.state = 3 # bin mapped   
        return True       
            
    def get_object_pose( self ):
        if self.hard_coded_object == True:
            print "####@@@####@@@ ATTENTION: Hard coded object pose in use ####@@@####@@@ "
            self.object_pose = HARD_CODED_OBJECT_POSITION
            self.object_pose.position = copy.deepcopy(self.configs[self.moving_arm][self.g_decision_making_state.bin_id]["gripper_config"].position)
            self.object_pose.position.x += random.uniform(  0.13, +0.25 )#.13
            self.object_pose.position.y += random.uniform( -0.05, +0.05 )#0
            self.object_pose.position.z += random.uniform( -0.06, -0.00 )#-.04
            print "The pose of the object is the following"
            print self.object_pose
            return self.object_pose
        for i in range(0, 10):
            self.object_pose = get_tf_transform(self.g_base_reference_frame, '/object_frame')
            if self.object_pose != False:
                #could be asking the ShelfDescription class for this.
                if self.object_pose.position.x < self.g_shelf_description.x:
                    print "======= Object detected outside (in front) of the bin"
                    print "object_pose.position.x = " + str(self.object_pose.position.x)
                    print "g_shelf_description.x " + str(self.g_shelf_description.x)
                else:
                    print "Detected object..."
                    print self.object_pose
                    return self.object_pose
        return False

    def move_closer_to_object( self, bin_id ):
        print "Trying to approach object"
        self.pre_grasp_sensing()
    
        if self.move_to_config( END_EFFECTOR, bin_id, 'gripper_config', ROBOT_SPEED_FAST) == True:    
            self.state = 4 # robot managed to move close to the bin
        else:
            print "######### Could not approach for grasping..."
            self.remove_object_from_scene(self.g_decision_making_state.object_name)
            self.state = 8 # robot fail to get close to object

    def evaluate_grasp( self ):
        print "Computing grasp..."

        if not self.compute_grasp_call():
            self.remove_object_from_scene(self.g_decision_making_state.object_name)
            self.state = 8
            return False

        if self.grasp_feedback["arm"] != self.moving_arm:
            print "In evaluate grasp: with arm feedback " + str(self.grasp_feedback["arm"]) + " and current arm " + str(self.moving_arm)
            print "The arm names are different - this should not have happened..."
            sys.exit(-1)
        else:   
            print "Grasp detected - proceeding to execution"
            self.state = 15 #Keep Arm
            
    def compute_grasp_call(self):
        rospy.wait_for_service('compute_grasp')
        req = compute_graspRequest();
        req.object_name = self.g_decision_making_state.object_name;
        print self.g_decision_making_state.object_name;
        try:
            compute_grasp_caller = rospy.ServiceProxy('compute_grasp', compute_grasp)
            resp = compute_grasp_caller(req);
        except rospy.ServiceException, e:
            print "Service call failed: "
        self.grasp_feedback = {}
        if resp.success:
            if resp.arm == "left":
                self.grasp_feedback["arm"] = LEFT_ARM
            else:
                self.grasp_feedback["arm"] = RIGHT_ARM
            return True
        else:
            print "Failed to return a valid grasp"
        return False

    def remove_object_from_scene(self,object_name):
        header = Header(stamp = rospy.Time.now(), frame_id = '/world')
        if "_attached" in object_name:
            return
        if object_name == "":
            return
        rospy.wait_for_service('clear_object')
        req = clear_objectRequest();
        req.object_name = object_name;
        try:
            clear_object_caller = rospy.ServiceProxy('clear_object', clear_object)
            resp1 = clear_object_caller(req);
        except rospy.ServiceException, e:
            print "Service call failed: "

    def pre_grasp_sensing(self):
        rospy.loginfo("##STARTING##");
        rospy.loginfo("Before grasp");
        rospy.sleep(2);
        if self.moving_arm ==0:
            arm = "left"
        else:
            arm = "right"
        if not SIMULATING:
            self.msgBeforeGrasp = rospy.wait_for_message("robotiq_force_torque_sensor_"+arm, ft_sensor);
        else:
            self.msgBeforeGrasp = ft_sensor()
            self.msgBeforeGrasp.Rf = 0
            return

        rospy.loginfo("Before grasp: %f",self.msgBeforeGrasp.Rf);

    def post_grasp_sensing(self):
        rospy.loginfo("After grasp");
        graspStatus = 2;
        rospy.sleep(2);
        if self.moving_arm ==0:
            arm = "left"
        else:
            arm = "right"
        if not SIMULATING:
            self.msgAfterGrasp = rospy.wait_for_message("robotiq_force_torque_sensor_"+arm, ft_sensor);
        else:
            self.msgAfterGrasp = ft_sensor()
            self.msgAfterGrasp.Rf = GRASP_THRESHOLD/2

        rospy.loginfo("After grasp: %f",self.msgAfterGrasp.Rf);

    def valid_grasp_check(self,expected_weight):
        #TODO here or from where function is called?
        #rfDiff before and after grasp to use for computation
        rfDiff = self.msgAfterGrasp.Rf - self.msgBeforeGrasp.Rf;

        #convert expectedWeight to Newtons
        rospy.loginfo("Expected weight in grams: %f", expected_weight);
        expectedForce = expected_weight * 0.00980665;
        rospy.loginfo("Expected force in newtons: %f", expectedForce);

        #if no object is sent will use threshold to make decision, else will use converted force within errors in measurement.
        if(expectedForce == 0):
            #if rfDiff > threshold too much force, if < 0, anamoly
            if(rfDiff > GRASP_THRESHOLD):
                rospy.loginfo("Without weight info: Looks like I'm carrying too much weight");
                return False;
            elif(rfDiff < 0 - GRASP_ERROR):
                rospy.loginfo("Without weight info: Anamoly. After weight < Before weight");
                return False;
            else:
                rospy.loginfo("Without weight info: Grasp looks good");
                return True;

        else:
            #if rfDiff within error of computed force, good; > weight, too heavy; < weight, too light
            if((rfDiff > expectedForce - GRASP_ERROR) and (rfDiff < expectedForce + GRASP_ERROR)):
                rospy.loginfo("With weight info: Grasp looks good");
                return True;
            elif(rfDiff <= expectedForce - GRASP_ERROR):
                rospy.loginfo("With weight info: Less than expected weight. Haven't picked up object or picked lighter object");
                return False;
            elif(rfDiff >= expectedForce + GRASP_ERROR):
                rospy.loginfo("With weight info: More than expected weight. Picked the heavier object or hit something");
                return False;

    def grasp(self,bin_id):
        print "Initiating Grasp"

        if self.moving_arm == RIGHT_ARM:
            self.right_gripper.open("bin_open")

        if self.moving_arm == LEFT_ARM:
            self.left_gripper.turn_on()

        # we need to go to the grasping configuration!!!
        if self.execute_plan( ROBOT_SPEED_SLOW, "grasping" ) == False:    
            print "######### Could not approach for grasping..."
            self.remove_object_from_scene(self.g_decision_making_state.object_name)
            self.state = 8 # robot fail to get close to object
            return False

        if self.moving_arm == RIGHT_ARM:
            self.right_gripper.close()

        # attach_object_to_the_gripper
        if self.g_object_attached_to_the_gripper == False:
            print "Attaching the object mesh to the gripper"
            grasp_svc = rospy.ServiceProxy("grasp", grasp)
            mesh_name = self.g_decision_making_state.object_name
            try:
                rospy.wait_for_service("grasp", 5.0)
                resp = grasp_svc(mesh_name)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("######### Service call failed: %s" % (e,))
                sys.exit(-1)  
            self.g_object_attached_to_the_gripper = True

        print "################ GRASPED!!"
        if self.execute_plan( ROBOT_SPEED_SLOW,"lifting") == False:
            print "Failed to lift"
            if self.moving_arm == RIGHT_ARM:
                self.right_gripper.open("bin_open")
            else:
                self.left_gripper.turn_off();
            self.dettach_object()
            self.state = 8 # robot fail to get close to object
            if self.move_to_config( END_EFFECTOR, bin_id, 'gripper_config', ROBOT_SPEED_SLOW) == False:    
                self.state = 13 # robot fail to get close to object
            return False

        self.state = 7 # grasped and raised
        return True

    def dettach_object(self):
        # dettach_object_from_the_gripper
        if self.g_object_attached_to_the_gripper == True:
            release_svc = rospy.ServiceProxy("release", grasp)
            try:
                rospy.wait_for_service("release", 5.0)
                if self.moving_arm == LEFT_ARM:
                    resp = release_svc("head_sponge")
                else:
                    resp = release_svc("arm_right_robotiq_virtual")
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("######### Service call failed: %s" % (e,))
                sys.exit(-1)    
                
            self.remove_object_from_scene( self.g_decision_making_state.object_name)
            self.g_object_attached_to_the_gripper = False

    def move_away_from_bin(self,bin_id):
        print "Try and retract the end effector with the object outside of the bin"
        # if self.move_to_config( END_EFFECTOR, bin_id, 'gripper_config', ROBOT_SPEED_SLOW ) == True:
        if self.execute_plan( ROBOT_SPEED_SLOW,"retracting" ) == True:

            #post-grasp sensing
            self.post_grasp_sensing()

            self.grasp_success = self.valid_grasp_check(0)
            print "########### Successfully grasped and retracted outside of bin"
            self.state = 5 # moved away from bin
        else:
            print "######### Could not retract #"
            if self.moving_arm == RIGHT_ARM:
                self.right_gripper.open("bin_open")
            else:
                self.left_gripper.turn_off();
            self.dettach_object()
            self.state = 8 # robot fail to move the object up
            if self.move_to_config( END_EFFECTOR, bin_id, 'gripper_config', ROBOT_SPEED_SLOW ) == False:
                self.state = 13 # moved away from bin

if __name__ == '__main__':
    tp = TaskPlanner()



                # if CALIBRATE_CAMERAS:
                #     while True:
                #         temp_bin = input("Bin? ")
                #         self.calib_bin = temp_bin
                #         if self.move_to_config( CAMERA, temp_bin, 'map_config', ROBOT_SPEED_FAST ) == False:
                #             print "Moving to configuration failed"
                #             sys.exit(-1)     
                #     sys.exit(-1)
