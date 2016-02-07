/*
 * decision_making.h
 *
 *  Created on: Dec 11, 2014
 *      Author: alberto
 */

#ifndef DECISION_MAKING_H_
#define DECISION_MAKING_H_

enum DECISION_MAKING_STATE
{
	Wait_for_Work_Order = 1,
	Select_Item = 2,
	Move_to_Bin = 3,
	Slam_Warm_Up = 9,
	Move_to_Detect1 = 10,
	Move_to_Detect2 = 11,
	Map_and_Detect_Object = 4,
	Evaluate_Grasp = 12,
	Move_closer_to_Object = 5,
	Grasp = 6,
	Move_Away_from_Bin = 7,
	Put_in_Order_Bin = 8
};


inline char *
get_state_name(int state)
{
	if (state == Wait_for_Work_Order) 	return ((char *) "Wait_for_Work_Order");
	if (state == Select_Item) 			return ((char *) "Select_Item");
	if (state == Move_to_Bin) 			return ((char *) "Move_to_Bin");
	if (state == Slam_Warm_Up) 			return ((char *) "Slam_Warm_Up");
	if (state == Move_to_Detect1) 		return ((char *) "Move_to_Detect1");
	if (state == Move_to_Detect2) 		return ((char *) "Move_to_Detect2");
	if (state == Evaluate_Grasp)        return ((char *) "Evaluate_Grasp");
	if (state == Map_and_Detect_Object) return ((char *) "Map_and_Detect_Object");
	if (state == Move_closer_to_Object) return ((char *) "Move_closer_to_Object");
	if (state == Grasp) 				return ((char *) "Grasp");
	if (state == Move_Away_from_Bin) 	return ((char *) "Move_Away_from_Bin");
	if (state == Put_in_Order_Bin) 		return ((char *) "Put_in_Order_Bin");
}


enum MODULES_MODES
{
	lspl_mode = 13,
	lstd_mode = 1,
	mstp_mode = 0,
	mspl_mode = 2,
	mstd_mode = 3,
	mfrz_mode = 4,
	ostd_mode = 5,
	mp_to_O = 6,
	mp_to_B = 7,
	mp_to_S = 14,
	mp_to_D1 = 15,
	mp_to_D2 = 16,
	mp_to_eval = 17,
	mp_to_M = 8,
	mp_to_Ob_ = 9,
	mp_to_AB_ = 10,
	mp_to_OB = 11,
	grasp = 12
};


enum MOTION_PLANNIG_ROBOT_STATE
{
	NONE = 0,
	AT_THE_ORIGIN = 1,
	AT_THE_BIN = 2,
	SLAM_WARMED_UP = 9,
	MOVED_TO_DETECT1 = 10,
	MOVED_TO_DETECT2 = 11,
	MAPPED = 3,
	CLOSE_TO_OBJECT = 4,
	AWAY_FROM_BIN = 5,
	AT_THE_ORDER_BIN = 6,
	GRASPED = 7,
	FAIL_TO_MOVE_TO_TARGET = 8,
	FAIL_TO_DETECT = 12,
	COLLISION_DETECTED = 13,
	KEEP_ARM = 15
};


enum AVAILABLE_ARMS
{
	LEFT_ARM = 0,
	RIGHT_ARM = 1
};

#endif /* DECISION_MAKING_H_ */
