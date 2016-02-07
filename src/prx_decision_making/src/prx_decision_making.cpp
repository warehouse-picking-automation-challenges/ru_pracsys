#include "ros/ros.h"
#include "prx_decision_making/DecisionMakingStateMessage.h"
#include "prx_decision_making/MotionPlanningStateMessage.h"
#include "prx_decision_making/decision_making.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
#include <ros/package.h>

//Json Parser
#include <json/read_file.h>

WorkOrder work_order;
BinContents bin_contents;
int current_item_index = 0;
int number_of_orders = 0;

// State Transition Triggers
bool Order_Arrived = false;
bool Got_Item = false;
bool At_the_Origin = false;
bool At_the_Bin = false;
bool Slam_Warmed_Up = false;
bool Moved_to_Detect1 = false;
bool Moved_to_Detect2 = false;
bool Mapped = false;
bool Detected = false;
bool Close_to_Object = false;
bool Grasped = false;
bool Away_from_Bin = false;
bool Item_in_Order_Bin = false;
bool Fail_to_Move_to_Target = false;

ros::Publisher *g_baxter_display_image_publisher;


int
init_decision_making_state_msg(prx_decision_making::DecisionMakingStateMessagePtr decision_making_state_msg)
{
    // decision_making_state_msg->header = ??;
    
    decision_making_state_msg->state = Wait_for_Work_Order;
    decision_making_state_msg->bin_id = 0;
    decision_making_state_msg->object_name = "";
    decision_making_state_msg->robot_arm = LEFT_ARM;
    
    decision_making_state_msg->motion_planning_mode = 0;
    decision_making_state_msg->grasp_planning_mode = 0;
    
    decision_making_state_msg->localization_mode = 0;
    decision_making_state_msg->mapping_mode = 0;
    decision_making_state_msg->object_pose_recognition_mode = 0;
}

void
clear_state_output(prx_decision_making::DecisionMakingStateMessagePtr decision_making_state_msg)
{
    decision_making_state_msg->motion_planning_mode = 0;
    decision_making_state_msg->grasp_planning_mode = 0;
    
    decision_making_state_msg->localization_mode = 0;
    decision_making_state_msg->mapping_mode = 0;
    decision_making_state_msg->object_pose_recognition_mode = 0;
}


bool
get_item_from_order(prx_decision_making::DecisionMakingStateMessagePtr& decision_making_state_msg)
{
    if (!Got_Item)
    {
        bool item_received = false;
        for (int i = current_item_index; i < number_of_orders + 1; ++i)
        {	
            if (work_order.done[i] == false)
            {	
                std::string bin_name = work_order.bin[i].substr(work_order.bin[i].length() - 1, 1);
                // std::cout << "bin_name: " << bin_name <<std::endl;
                decision_making_state_msg->bin_id = (int)(bin_name[0]);
                decision_making_state_msg->object_name = work_order.item[i].c_str();
//				if (((decision_making_state_msg->bin_id - 'A' + 1) % 3) == 0)
//				    decision_making_state_msg->robot_arm = RIGHT_ARM;
//				else
                decision_making_state_msg->robot_arm = LEFT_ARM;
                current_item_index = i;
                item_received = true;
                break;
            }
        }
        if (item_received == false)
        {
            std::cout << "Done with all items." <<std::endl;
            std::cin.get();
            return (false); // If there are no more items, return false.
        }
        std::cout << "bin_id: " << (char) decision_making_state_msg->bin_id <<std::endl;
        std::cout << "object_name: " <<decision_making_state_msg->object_name <<std::endl;
        std::cout << "robot_arm: " <<decision_making_state_msg->robot_arm <<std::endl;
        return (true);
    }
    else
        return (true);
}


void
display_system_state_on_baxter_screen(prx_decision_making::DecisionMakingStateMessagePtr& decision_making_state_msg)
{
	std::stringstream file_name;
	static int previous_state = -1;

	if (previous_state != decision_making_state_msg->state)
	{
		previous_state = decision_making_state_msg->state;
		std::string path = ros::package::getPath("prx_decision_making");
		file_name << path << "/images/state_machine-" << decision_making_state_msg->state << ".png";
		// std::cout << "================ " << file_name.str().c_str() << std::endl;
		cv::Mat image = cv::imread(file_name.str().c_str(), CV_LOAD_IMAGE_COLOR);
		if (image.data)
		{
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
			g_baxter_display_image_publisher->publish(msg);
		}
		else
			printf("Error: Could not open image file %s in display_system_state_on_baxter_screen()\n", file_name.str().c_str());
	}
}


int
perform_state_action(prx_decision_making::DecisionMakingStateMessagePtr& decision_making_state_msg)
{
	// State Action
	switch (decision_making_state_msg->state)
	{
		case Wait_for_Work_Order:
			Order_Arrived = true; // Check if file with order exists
			break;
		case Select_Item:
			Got_Item = get_item_from_order(decision_making_state_msg);
			break;
		case Move_to_Bin:
			Got_Item = false;
			break;
		case Slam_Warm_Up:
			break;
		case Move_to_Detect1:
			break;
		case Move_to_Detect2:
			break;
		case Map_and_Detect_Object:
			break;
		case Move_closer_to_Object:
			break;
		case Grasp:
			break;
		case Move_Away_from_Bin:
			break;
		case Put_in_Order_Bin:
			break;
		default:
			printf("Error: Unknown state in perform_state_action()\n");
			return (1);
	}
	//display_system_state_on_baxter_screen(decision_making_state_msg);

	return (0);
}


int
perform_state_transition(prx_decision_making::DecisionMakingStateMessagePtr& decision_making_state_msg)
{
	// State Transition
	switch (decision_making_state_msg->state)
	{
		case Wait_for_Work_Order:
			if (Order_Arrived)
				decision_making_state_msg->state = Select_Item;
			break;
		case Select_Item:
			if (Got_Item && At_the_Origin)
				decision_making_state_msg->state = Move_to_Bin;
//				decision_making_state_msg->state = Map_and_Detect_Object;
			else if (!Got_Item)
				decision_making_state_msg->state = Wait_for_Work_Order;
			break;
		case Move_to_Bin:
			if (At_the_Bin)
				decision_making_state_msg->state = Slam_Warm_Up;
			break;
		case Slam_Warm_Up:
			if (Slam_Warmed_Up)
				decision_making_state_msg->state = Move_to_Detect1;
			break;
		case Move_to_Detect1:
			if (Moved_to_Detect1)
				decision_making_state_msg->state = Move_to_Detect2;
			break;
		case Move_to_Detect2:
			if (Moved_to_Detect2)
				decision_making_state_msg->state = Map_and_Detect_Object;
			break;
		case Map_and_Detect_Object:
			if (Mapped)// && Detected)
				decision_making_state_msg->state = Move_closer_to_Object;
			break;
		case Move_closer_to_Object:
			if (Close_to_Object)
				decision_making_state_msg->state = Grasp;
			if (Fail_to_Move_to_Target)
				decision_making_state_msg->state = Move_to_Bin;
			break;
		case Grasp:
			if (Grasped)
				decision_making_state_msg->state = Move_Away_from_Bin;
			if (Fail_to_Move_to_Target)
				decision_making_state_msg->state = Move_to_Bin;
			break;
		case Move_Away_from_Bin:
			if (Away_from_Bin)
			{
				decision_making_state_msg->state = Put_in_Order_Bin;
				work_order.done[current_item_index] = true;
			}
			if (Fail_to_Move_to_Target)
				decision_making_state_msg->state = Move_to_Bin;
			break;
		case Put_in_Order_Bin:
			if (Item_in_Order_Bin)
				decision_making_state_msg->state = Select_Item;
			break;
		default:
			printf("Error: Unknown state in perform_state_transition()\n");
			return (2);
	}
	return (0);
}


int
generate_state_output(prx_decision_making::DecisionMakingStateMessagePtr& decision_making_state_msg)
{
	// State Output
	clear_state_output(decision_making_state_msg);
	switch (decision_making_state_msg->state)
	{
		case Wait_for_Work_Order:
			break;
		case Select_Item:
			decision_making_state_msg->motion_planning_mode = mp_to_O;
			decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
            // decision_making_state_msg->mapping_mode = mspl_mode;
			break;
		case Move_to_Bin:
			// Bin_id set in Select_Item state
			decision_making_state_msg->localization_mode = lspl_mode;
			decision_making_state_msg->mapping_mode = mstp_mode;
			decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
			decision_making_state_msg->motion_planning_mode = mp_to_B;
			break;
		case Slam_Warm_Up:
			// Bin_id set in Select_Item state
			decision_making_state_msg->localization_mode = lspl_mode;
			decision_making_state_msg->mapping_mode = mspl_mode;
			decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
			decision_making_state_msg->motion_planning_mode = mp_to_S;
			break;
		case Move_to_Detect1:
			// Bin_id set in Select_Item state
			decision_making_state_msg->localization_mode = lspl_mode;
			decision_making_state_msg->mapping_mode = mstd_mode;
			decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
			decision_making_state_msg->motion_planning_mode = mp_to_D1;
			break;
		case Move_to_Detect2:
			// Bin_id set in Select_Item state
			decision_making_state_msg->localization_mode = lspl_mode;
			decision_making_state_msg->mapping_mode = mstd_mode;
			decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
			decision_making_state_msg->motion_planning_mode = mp_to_D2;
			break;
		case Map_and_Detect_Object:
			// Object_id set in Select_Item state
			decision_making_state_msg->localization_mode = lspl_mode;
			decision_making_state_msg->mapping_mode = mstd_mode;
			decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
			decision_making_state_msg->motion_planning_mode = mp_to_M;
			break;
		case Move_closer_to_Object:
			// Object_id set in Select_Item state
			decision_making_state_msg->localization_mode = lstd_mode;
			decision_making_state_msg->mapping_mode = mfrz_mode;
			decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
			decision_making_state_msg->motion_planning_mode = mp_to_Ob_;
			break;
		case Grasp:
			// Object_id set in Select_Item state
			decision_making_state_msg->localization_mode = lstd_mode;
			decision_making_state_msg->mapping_mode = mstp_mode;
			decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
			// decision_making_state_msg->grasp_planning_mode = grasp;
			decision_making_state_msg->motion_planning_mode = grasp;
			break;
		case Move_Away_from_Bin:
			decision_making_state_msg->localization_mode = lstd_mode;
			decision_making_state_msg->mapping_mode = mstp_mode;
			decision_making_state_msg->motion_planning_mode = mp_to_AB_;
			break;
		case Put_in_Order_Bin:
			decision_making_state_msg->motion_planning_mode = mp_to_OB;
			break;
		default:
			printf("Error: Unknown state in generate_state_output()\n");
			return (3);
	}
	return (0);
}


int
run_decision_making_state_machine(prx_decision_making::DecisionMakingStateMessagePtr decision_making_state_msg)
{
	int error;

	error = perform_state_action(decision_making_state_msg);
	if (error != 0)
		return (error);

	error = perform_state_transition(decision_making_state_msg);
	if (error != 0)
		return (error);

	error = generate_state_output(decision_making_state_msg);
	if (error != 0)
		return (error);

	return (0);
}


void
motion_planner_subscriber_callback(prx_decision_making::MotionPlanningStateMessagePtr robot_state)
{
	At_the_Origin = At_the_Bin = Slam_Warmed_Up = Moved_to_Detect1 = Moved_to_Detect2 = Mapped = false;
	Close_to_Object = Grasped = Away_from_Bin = Item_in_Order_Bin = Fail_to_Move_to_Target = false;

	if (robot_state->state == AT_THE_ORIGIN)
		At_the_Origin = true;
	else if (robot_state->state == AT_THE_BIN)
		At_the_Bin = true;
	else if (robot_state->state == SLAM_WARMED_UP)
		Slam_Warmed_Up = true;
	else if (robot_state->state == MOVED_TO_DETECT1)
		Moved_to_Detect1 = true;
	else if (robot_state->state == MOVED_TO_DETECT2)
		Moved_to_Detect2 = true;
	else if (robot_state->state == MAPPED)
		Mapped = true;
	else if (robot_state->state == CLOSE_TO_OBJECT)
		Close_to_Object = true;
	else if (robot_state->state == GRASPED)
		Grasped = true;
	else if (robot_state->state == AWAY_FROM_BIN)
		Away_from_Bin = true;
	else if (robot_state->state == AT_THE_ORDER_BIN)
		Item_in_Order_Bin = true;	
	else if (robot_state->state == FAIL_TO_MOVE_TO_TARGET)
		Fail_to_Move_to_Target = true;
}


int
main(int argc, char **argv)
{
	ros::init(argc, argv, "prx_decision_making");
	ros::NodeHandle nh;

   	if (argc != 2)
   	{
   		std::cout << "Usage:\n prx_decision_making_node <work order Json file>" << std::endl;
                
                return (1);
        }

   	std::string work_order_json_file_name(argv[1]);
	number_of_orders = read_file(work_order_json_file_name, work_order, bin_contents);

	// Publishers
	ros::Publisher decison_making_state_publisher =
		nh.advertise<prx_decision_making::DecisionMakingStateMessage>("decision_making_state", 1);
	ros::Publisher baxter_display_image_publisher = nh.advertise<sensor_msgs::Image>("/robot/xdisplay", 1, true);
	g_baxter_display_image_publisher = &baxter_display_image_publisher;

	//Subscribers
	ros::Subscriber motion_planner_subscriber =
		nh.subscribe("/motion_planner_state", 1, motion_planner_subscriber_callback);

	ros::Rate loop_rate(40);

	prx_decision_making::DecisionMakingStateMessagePtr decision_making_state_msg(new prx_decision_making::DecisionMakingStateMessage);
	init_decision_making_state_msg(decision_making_state_msg);

	std::cout << "\n";
	int error = 0;
	static int previous_state = -1;
	while (ros::ok() && (error == 0))
	{
		decison_making_state_publisher.publish(decision_making_state_msg);

		error = run_decision_making_state_machine(decision_making_state_msg);
		if (decision_making_state_msg->state != previous_state)
		{
			std::cout << get_state_name(decision_making_state_msg->state) << "                           \n";
			previous_state = decision_making_state_msg->state;
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
