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
ItemPreferences item_preferences;
int current_item_index = 0;
int number_of_orders = 0;

// State Transition Triggers
bool Order_Arrived = false;
bool Got_Item = false;
bool At_the_Origin = false;
bool At_the_Bin = false;
bool Mapped = false;
bool Close_to_Object = false;
bool Grasped = false;
bool Away_from_Bin = false;
bool Item_in_Order_Bin = false;
bool Fail_to_Move_to_Target = false;
bool Slam_Warmed_Up = false;
bool Moved_to_Detect1 = false;
bool Moved_to_Detect2 = false;
bool Keep_arm = false;

bool Collision_Detected = false;
bool Fail_to_Detect = false;

bool g_data_gathering = false;


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
        if( work_order.remaining_items == 0)
    	{
            std::cout << "Done with all items." <<std::endl;
            std::cin.get();
            exit(0); // If there are no more items, exit.
    	}

        while (work_order.done[current_item_index] == true)
    	{
            ROS_INFO_STREAM("index: "<< current_item_index <<" complete ");
            current_item_index = (current_item_index + 1) % (number_of_orders+1);
        }

        std::string bin_name = work_order.bin[current_item_index].substr(work_order.bin[current_item_index].length() - 1, 1);
        ROS_INFO_STREAM("bin_name: "<< bin_name <<" Item index: "<<current_item_index);
        decision_making_state_msg->bin_id = (int)(bin_name[0]);
        decision_making_state_msg->item_counts = work_order.item_counts[current_item_index];
        decision_making_state_msg->object_name = work_order.item[current_item_index].c_str();
        decision_making_state_msg->robot_arm = ! (work_order.gripper[current_item_index] % 2);
        ROS_INFO_STREAM("for item " << work_order.item[current_item_index] << " i will use arm " << decision_making_state_msg->robot_arm);
    			
	
        ROS_INFO_STREAM("bin_id: " << (char) decision_making_state_msg->bin_id );
        ROS_INFO_STREAM("object_name: " <<decision_making_state_msg->object_name );
        ROS_INFO_STREAM("robot_arm: " <<decision_making_state_msg->robot_arm );
        return (true);
    }
    else
        return (true);
}

int
perform_state_action(prx_decision_making::DecisionMakingStateMessagePtr& decision_making_state_msg)
{
    // State Action
    switch (decision_making_state_msg->state)
    {
    case Select_Item:
        Got_Item = get_item_from_order(decision_making_state_msg);
        break;
    case Move_to_Bin:
        Got_Item = false;
        break;
    case Wait_for_Work_Order:
    case Slam_Warm_Up:
    case Move_to_Detect1:
    case Move_to_Detect2:
    case Map_and_Detect_Object:
    case Evaluate_Grasp:
    case Move_closer_to_Object:
    case Move_Away_from_Bin:
    case Grasp:
    case Put_in_Order_Bin:
        break;
    default:
        printf("Error: Unknown state in perform_state_action()\n");
        return (1);
    }
    
    return (0);
}

void progress_due_to_failure( prx_decision_making::DecisionMakingStateMessagePtr& decision_making_state_msg )
{
    current_item_index = (current_item_index + 1) % (number_of_orders+1);
    ROS_INFO_STREAM("Failed, updating index to "<<current_item_index);
    decision_making_state_msg->state = Select_Item;
}

int
perform_state_transition(prx_decision_making::DecisionMakingStateMessagePtr& decision_making_state_msg)
{
    // State Transition
    if (Collision_Detected)
    {
        if (g_data_gathering)
        {
            std::cout << "###################### Planning failure!!! Exiting..." << std::endl;
            exit(1);
        }
        progress_due_to_failure( decision_making_state_msg );
    }
    else
    {
        switch (decision_making_state_msg->state)
        {
        case Wait_for_Work_Order:
            decision_making_state_msg->state = Select_Item;
            break;
        case Select_Item:
            if (At_the_Origin || g_data_gathering)
                decision_making_state_msg->state = Move_to_Bin;
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
            if (g_data_gathering && Mapped)
            {
                work_order.done[current_item_index] = true;
                work_order.remaining_items--;
                decision_making_state_msg->state = Select_Item;
            }
            else if (Fail_to_Detect)
                progress_due_to_failure( decision_making_state_msg );
            else if (Mapped)
                decision_making_state_msg->state = Move_closer_to_Object;
            break;
        case Move_closer_to_Object:
            if (Fail_to_Move_to_Target)
                progress_due_to_failure( decision_making_state_msg );
            else if (Close_to_Object)
                decision_making_state_msg->state = Evaluate_Grasp;
            break;
        case Evaluate_Grasp:
            if (Fail_to_Move_to_Target)
                progress_due_to_failure( decision_making_state_msg );
            else if (Keep_arm)
                decision_making_state_msg->state = Grasp;
            break;
        case Grasp:
            if (Fail_to_Move_to_Target)
                progress_due_to_failure( decision_making_state_msg );
            else if (Grasped)
                decision_making_state_msg->state = Move_Away_from_Bin;
            break;
        case Move_Away_from_Bin:
            if (Away_from_Bin)
                decision_making_state_msg->state = Put_in_Order_Bin;
            if (Fail_to_Move_to_Target)
                progress_due_to_failure( decision_making_state_msg );
            break;
        case Put_in_Order_Bin:
            if (Item_in_Order_Bin)
            {
                // work_order.done[current_item_index] = true;
                // work_order.remaining_items--;
                current_item_index = (current_item_index + 1) % (number_of_orders+1);
                decision_making_state_msg->state = Select_Item;
            }
            break;
        default:
            printf("Error: Unknown state in perform_state_transition()\n");
            return (2);
        }
    }
    Collision_Detected = Fail_to_Detect = At_the_Origin = At_the_Bin = Slam_Warmed_Up = Moved_to_Detect1 = Moved_to_Detect2 = false;
    Mapped = Close_to_Object = Grasped = Item_in_Order_Bin = Fail_to_Move_to_Target= Away_from_Bin =Keep_arm = false;
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
        if (!g_data_gathering)
            decision_making_state_msg->motion_planning_mode = mp_to_O;  // move to the origin
        break;
    case Move_to_Bin:
        // Bin_id set in Select_Item state
        decision_making_state_msg->localization_mode = lspl_mode;
        decision_making_state_msg->mapping_mode = mstp_mode;   // stop
        decision_making_state_msg->motion_planning_mode = mp_to_B;
        break;
    case Slam_Warm_Up:
        // Bin_id set in Select_Item state
        decision_making_state_msg->localization_mode = lspl_mode;
        decision_making_state_msg->mapping_mode = mspl_mode;   // THIS NEEDS TO BE MSPL_MODE
        decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
        decision_making_state_msg->motion_planning_mode = mp_to_S;
        break;
    case Move_to_Detect1:
        // Bin_id set in Select_Item state
        decision_making_state_msg->localization_mode = lspl_mode;
        decision_making_state_msg->mapping_mode = mspl_mode;
        decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
        decision_making_state_msg->motion_planning_mode = mp_to_D1;
        break;
    case Move_to_Detect2:
        // Bin_id set in Select_Item state
        decision_making_state_msg->localization_mode = lspl_mode;
        decision_making_state_msg->mapping_mode = mspl_mode;
        decision_making_state_msg->object_pose_recognition_mode = ostd_mode;
        decision_making_state_msg->motion_planning_mode = mp_to_D2;
        break;
    case Map_and_Detect_Object:
        // Object_id set in Select_Item state
        decision_making_state_msg->localization_mode = lspl_mode;
        decision_making_state_msg->mapping_mode = mstd_mode;
        decision_making_state_msg->object_pose_recognition_mode = ostd_mode;   // needs to change to robust estimation mode
        decision_making_state_msg->motion_planning_mode = mp_to_M;
        break;
    case Move_closer_to_Object:
        // Object_id set in Select_Item state
        decision_making_state_msg->localization_mode = lstd_mode;    // stop
        decision_making_state_msg->mapping_mode = mstp_mode;         // stop
        decision_making_state_msg->motion_planning_mode = mp_to_Ob_;
        break;
    case Evaluate_Grasp:
        // Object_id set in Select_Item state
        decision_making_state_msg->localization_mode = lstd_mode;    // stop       
        decision_making_state_msg->motion_planning_mode = mp_to_eval;
        break;
    case Grasp:
        // Object_id set in Select_Item state
        decision_making_state_msg->localization_mode = lstd_mode;    // stop
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
task_planner_subscriber_callback(prx_decision_making::MotionPlanningStateMessagePtr robot_state)
{
    Collision_Detected = Fail_to_Detect = At_the_Origin = At_the_Bin = Slam_Warmed_Up = Moved_to_Detect1 = Moved_to_Detect2 = false;
    Mapped = Close_to_Object = Grasped = Item_in_Order_Bin = Fail_to_Move_to_Target= Away_from_Bin =Keep_arm = false;
    
    if (robot_state->state == AT_THE_ORIGIN)
    {
        std::cout<<"\n At the origin"<<std::endl;
        At_the_Origin = true;
    }
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
    else if (robot_state->state == KEEP_ARM)
        Keep_arm = true;
    else if (robot_state->state == GRASPED)
        Grasped = true;
    else if (robot_state->state == AWAY_FROM_BIN)
        Away_from_Bin = true;
    else if (robot_state->state == AT_THE_ORDER_BIN)
        Item_in_Order_Bin = true;	
    else if (robot_state->state == FAIL_TO_DETECT )
        Fail_to_Detect = true;
    else if (robot_state->state == FAIL_TO_MOVE_TO_TARGET)
        Fail_to_Move_to_Target = true;
    else if (robot_state->state == COLLISION_DETECTED )
        Collision_Detected = true;
}


void stage( WorkOrder& new_work_order, WorkOrder& work_order, ItemPreferences& item_preferences, BinContents& bin_contents, 
            int input_priority, bool single_item_bin, bool gripper_switch, int& item_count )
{
    if (g_data_gathering) {
        std::cout << "entered" << std::endl;
        for (int i= 0; i < number_of_orders+1; i++) {
            std::cout << "i: " << i << std::endl;
            new_work_order.bin[item_count] = work_order.bin[i];
            new_work_order.item[item_count] = work_order.item[i];
            new_work_order.item_counts[item_count] = work_order.item_counts[i];
            new_work_order.gripper[item_count] = (PREFERENCE) 1;
            new_work_order.done[item_count] = false;
            item_count++;
        }
    }
    else {
        for(int i=0;i<25;i++)
        {
            std::string item_name = item_preferences.items[i];
            if(item_preferences.priority[i]==input_priority)
            {
                for(int j=0;j<number_of_orders+1;j++)
                {
                    // std::cout<<work_order.item[j]<<" "<<item_name<<std::endl;
                    if( (single_item_bin && work_order.item_counts[j]==1) ||
                        (!single_item_bin && work_order.item_counts[j]>1) )
                    {
                        if(work_order.item[j] == item_name)
                        {
                            if(!gripper_switch || item_preferences.gripper[i] >= 2)
                            {
                                // std::cout<<"---OK"<<std::endl;
                                new_work_order.bin[item_count] = work_order.bin[j];
                                new_work_order.item[item_count] = work_order.item[j];
                                new_work_order.item_counts[item_count] = work_order.item_counts[j];
                                if(gripper_switch)
                                {
                                    new_work_order.gripper[item_count] = (PREFERENCE) (item_preferences.gripper[i]==3?2:3);
                                }
                                else
                                {
                                    new_work_order.gripper[item_count] = item_preferences.gripper[i];
                                }
                                new_work_order.done[item_count] = false;
                                item_count++;
                            }
                        }
                    }
                }
            }          
        }
    }
}

void new_work_order_generation(WorkOrder& work_order, ItemPreferences& item_preferences, BinContents& bin_contents)
{
    WorkOrder new_work_order;
    int item_count = 0;
    //resequence order
    if (g_data_gathering) {
        stage(new_work_order,work_order,item_preferences,bin_contents,0,true,false,item_count);
    }
    else {
        stage(new_work_order,work_order,item_preferences,bin_contents,0,true,false,item_count);
        stage(new_work_order,work_order,item_preferences,bin_contents,0,false,false,item_count);
        stage(new_work_order,work_order,item_preferences,bin_contents,0,true,true,item_count);
        stage(new_work_order,work_order,item_preferences,bin_contents,0,false,true,item_count);
        stage(new_work_order,work_order,item_preferences,bin_contents,1,true,false,item_count);
        stage(new_work_order,work_order,item_preferences,bin_contents,1,false,false,item_count);
        stage(new_work_order,work_order,item_preferences,bin_contents,1,true,true,item_count);
        stage(new_work_order,work_order,item_preferences,bin_contents,1,false,true,item_count);
    }
    new_work_order.remaining_items = item_count;
    work_order = new_work_order;
    number_of_orders = item_count-1;
}

void
get_data_gathering_flag()
{
    if (ros::param::get("/data_gathering", g_data_gathering))
    {
        std::cout << "data_gathering: " << std::boolalpha << g_data_gathering << std::endl;
    }
    else
    {
        std::cout << "No data_gathering flag specified. Assuming false." << std::endl;
        g_data_gathering = false;
    }
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "prx_decision_making");
    ros::NodeHandle nh;

    get_data_gathering_flag();

    if (argc != 2)
    {
         ROS_FATAL_STREAM("Usage:\n prx_decision_making_node <work order Json file>" );
        
      	return (1);
    }
    
    char* w = std::getenv("PRACSYS_PATH");
    std::string dir(w);
    dir += ("/../");
    std::string work_order_json_file_name(argv[1]);
    work_order_json_file_name = dir+work_order_json_file_name;
    std::string item_preferences_file_name = dir+"grasping_preferences_icra.txt";
    number_of_orders = read_file(work_order_json_file_name, work_order, bin_contents);
    read_preferences(item_preferences_file_name, item_preferences);

    count_items_in_bins(bin_contents,work_order,number_of_orders+1);
    for(int i=0;i<12;i++)
    {
        ROS_INFO_STREAM("Items in "<<i<<": "<<bin_contents.item_counts[i]);
    }

    new_work_order_generation(work_order,item_preferences,bin_contents);
    
    for(int j=0;j<number_of_orders+1;j++)
    {
        ROS_INFO_STREAM(work_order.bin[j]<<" "<<work_order.item[j]<<" "<<work_order.item_counts[j]<<" "<<work_order.gripper[j]);
    }
    
    // Publishers
    ros::Publisher decision_making_state_publisher =
        nh.advertise<prx_decision_making::DecisionMakingStateMessage>("decision_making_state", 1);
    
    //Subscribers
    ros::Subscriber task_planner_subscriber =
        nh.subscribe("/task_planner_state", 1, task_planner_subscriber_callback);
    ros::Subscriber task_planner_subscriber2 =
        nh.subscribe("/motion_planner_state", 1, task_planner_subscriber_callback);
    
    ros::Rate loop_rate(15);
    
    prx_decision_making::DecisionMakingStateMessagePtr decision_making_state_msg(new prx_decision_making::DecisionMakingStateMessage);
    init_decision_making_state_msg(decision_making_state_msg);

    std::cout << "\n";
    int error = 0;
    static int previous_state = -1;
    while (ros::ok() && (error == 0))
    {
        decision_making_state_publisher.publish(decision_making_state_msg);
        
        error = run_decision_making_state_machine(decision_making_state_msg);
        if (decision_making_state_msg->state != previous_state)
        {
            ROS_INFO_STREAM(get_state_name(decision_making_state_msg->state) << "\n");
            previous_state = decision_making_state_msg->state;
        }
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }
    
    return 0;
}
