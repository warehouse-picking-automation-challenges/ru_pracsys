
/**
 * @file base_apc_planning_application.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/applications/planning_application.hpp"
#include "simulation/plants/manipulator.hpp"
#include "simulation/plants/movable_body_plant.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/system_graph.hpp"
#include "prx/planning/task_planners/base_apc_task_planner/base_apc_task_query.hpp"
#include "prx/planning/modules/IK_data_base/IK_data_base.hpp"
#include "prx_planning/compute_plan.h"
#include "prx_planning/execute_plan.h"
#include "prx_planning/place_object.h"
#include "prx_planning/clear_objects.h"
#include "prx_planning/grasp.h"
#include "prx_planning/release.h"
#include "prx_planning/validate_end_effector.h"
#include "prx_planning/clear_object.h"
#include "prx_planning/compute_grasp.h"
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <actionlib/client/simple_action_client.h>
#include "prx/utilities/definitions/sys_clock.hpp"

#include <tf/transform_listener.h>

#ifndef PRX_MOTOMAN_APC_APPLICATION_HPP
#define	PRX_MOTOMAN_APC_APPLICATION_HPP

#include <prx_planning/compute_planAction.h>
#include <prx_planning/execute_planAction.h>
#include <actionlib/server/simple_action_server.h>

#include <semaphore.h>
namespace prx
{
    namespace plan
    {

        struct trajectory_db_entry
        {
            util::space_point_t* start;
            util::config_t end_configuration;
            sim::trajectory_t trajectory;
        };

        typedef actionlib::SimpleActionServer<prx_planning::compute_planAction> CServer;
        typedef actionlib::SimpleActionServer<prx_planning::execute_planAction> EServer;
        class motoman_planning_application_t : public planning_application_t
        {
        public:
            motoman_planning_application_t();
            virtual ~motoman_planning_application_t();

            /**
             * @copydoc planning_application_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader);

            /**
             * @copydoc planning_application_t::execute()
             *
             */
            virtual void execute();

          protected:

            sem_t semaphore;
            
            //Let's then create a list of end-effector configurations which are properly transformed
            std::vector< util::config_t > grasp_configurations;

            void stop_updating_state()
            {
                  sem_wait(&semaphore);
            }     
            void start_updating_state()
            {
                  sem_post(&semaphore);
            }

            /********************************************/
            /***  CALLBACKS                **************/
            /********************************************/

            // Forwards a previously computed plan to the robot
            // bool process_get_plan_callback(prx_planning::compute_plan::Request& req, prx_planning::compute_plan::Response& res );
            void compute_plan_server_callback(const prx_planning::compute_planGoalConstPtr& req);

            // Forwards a previously computed plan to the robot
            // bool process_execute_plan_callback(prx_planning::execute_plan::Request& req, prx_planning::execute_plan::Response& res );
            void execute_plan_server_callback(const prx_planning::execute_planGoalConstPtr& req);

            // Removes all the objects from the scene
            bool clear_objects_callback(prx_planning::clear_objects::Request& req, prx_planning::clear_objects::Response& res );

            // Removes an object or an obstacle (order bin or shelf) from the scene
            bool clear_object_callback(prx_planning::clear_object::Request& req, prx_planning::clear_object::Response& res );

            // Activate an object and place it where the pose estimation is specifying
            virtual bool place_object_callback(prx_planning::place_object::Request& req, prx_planning::place_object::Response& res );

            void place_albertos_box( bool activate );

            // We are informed that a grasp is being executed
            bool grasp_callback(prx_planning::grasp::Request& req, prx_planning::grasp::Response& res );

            // We are informed that an object is being released
            bool release_callback(prx_planning::release::Request& req, prx_planning::release::Response& res );

            // Provide a valid, collision-free IK solution for an end-effector pose
            bool validate_end_effector_callback(prx_planning::validate_end_effector::Request& req, prx_planning::validate_end_effector::Response& res );

            // Compute a valid grasp configuration for a given object pose
            bool compute_grasp_callback(prx_planning::compute_grasp::Request& req, prx_planning::compute_grasp::Response& res );

            // Receive the state of the manipulator from the robot
            void get_state_callback( const sensor_msgs::JointState& stateMsg );
            
            // Initiate planning to find grasp positions for the specified object and attach these to the roadmap
            bool find_grasp_for_object( );

            /********************************************/
            /***  PROTECTED FUNCTIONS      **************/
            /********************************************/

            // An initialization function which will load in all of the data we have computed
            // for grasps.
            void load_grasp_data(const util::parameter_reader_t* reader);

            // A function for broadcasting tf transforms so that RViz properly visualizes
            // the shelf and the objects in the environment.
            void tf_broadcasting();

            // Enable the robot so that it executes trajectories
            bool communicate_trajectory( sim::trajectory_t& traj, double speed );

            // Compute roadmaps if not loading them - also dealing with database
            void compute_roadmaps();

            void load_trajectories();

            bool check_for_stored_trajectory( util::space_point_t* source_state, util::config_t ee_config );

            // Initialize object geometry for an object that has just been activated
            void init_object_geometry( std::string object_name );

            // Check whether it is necessary to change the context depending on which arm must move
            void check_for_context_switch( std::string the_arm );

            // IK-type function which finds REACHABLE, COLLISION-FREE, and VALID IK solutions for the given end-effector configuration
            bool reachable_IK( bool camera_link, util::space_point_t* source_state, const util::config_t& ee_config, int IK_stage );


            // Generating seeds for IK
            void generate_IK_seeds( bool for_camera = false );

            // Detecting collision-free and valid IK solutions
            int generate_valid_IK_states( bool camera_link, const util::config_t& config, int IK_stage );

            // Given the list of collision-free/valid IKs, find the reachable ones
            bool compute_reachable_IK( util::space_point_t* source_state, const util::config_t& ee_config, int IK_stage, int number_of_valid_states );

            // copying over from current state the appropriate variables
            void impose_hand_state( util::space_point_t* target_state );

            // Internal path-finding call with removal from the bin for grasping tasks
            bool IK_recursion( util::space_point_t* destination, const util::config_t& destination_config, int IK_stage );

            bool check_IK_steering( bool camera_link, util::space_point_t* source_state, const util::config_t& ee_config, int IK_stage );

            base_apc_task_query_t*  setup_planning( util::space_point_t* source_state, int IK_stage );

            // Internal path-finding call.
            bool find_path_internal( util::space_point_t* source, util::space_point_t* destination, int IK_stage );

            // Internal path-finding call multi-target
            bool find_shortest_path_internal( util::space_point_t* source, std::vector<util::space_point_t*> destinations, int IK_stage = 0 );

            // Add trajectory to the database
            void add_trajectory_to_database( util::space_point_t* start, util::config_t ee_config, sim::trajectory_t& traj );

            /********************************************/
            /***  MESSAGE HANDLING               ********/
            /********************************************/

            // The handle to this ROS node
            ros::NodeHandle n;
            // Forwards a previously computed plan to the robot
            // Request: An end state (15 numbers) and a hand type (boolean)
            // Response: Whether a plan has been found (boolean)
            ros::ServiceServer get_plan_service;
            // Forwards a previously computed plan to the robot
            // Request: The speed of execution (float), the higher it is the faster the arm moves
            // Response: Whether we succeeded or not (boolean)
            ros::ServiceServer execute_plan_service;
            // Removes all the objects from the scene
            // Request: empty
            // Response: Whether we succeeded or not (boolean)
            ros::ServiceServer clear_objects_service;
            // Removes an object or an obstacle (order bin or shelf) from the scene
            // Request: An object name (string)
            // Response: Whether we succeeded or not (boolean)
            ros::ServiceServer clear_object_service;
            // Activate an object and place it where the pose estimation is specifying
            // Request: An object name (string) and a pose
            // Response: Whether we succeeded or not (boolean)
            ros::ServiceServer place_object_service;
            // We are informed that a grasp is being executed
            // Request: An object name (string) to be grasped
            // Response: Whether we succeeded or not (boolean)
            ros::ServiceServer grasp_service;
            // We are informed that an object is being released
            // Request: An object name (string) to be released
            // Response: Whether we succeeded or not (boolean)
            ros::ServiceServer release_service;
            // Provide a valid, collision-free IK solution for an end-effector pose
            // Request: The desired end-effector pose and the arm to be moved (string)
            // Response: The arm configuration and whether we succeeded or not (boolean)
            ros::ServiceServer validate_end_effector_service;

            ros::ServiceServer compute_grasp_service;

            CServer* compute_plan_server;
            EServer* execute_plan_server;

            // Publishing geometry information to RViz
            ros::Publisher vis_array_pub;
            ros::Publisher cancel_publisher;

            // Publishes control messages to the robot
            // It contains a list of names for the joints
            // And then it has a list of joint trajectory points 
            // A joint trajectory point is a state and a time stamp
            control_msgs::FollowJointTrajectoryGoal motoman_command;
            control_msgs::FollowJointTrajectoryGoal unigripper_command;
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac;
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac_uni;


            // Subscriber to the joint state of the robot
            ros::Subscriber sub_joint;

            /********************************************/
            /***  PROTECTED LOCAL VARIABLES      ********/
            /********************************************/

            std::vector<double> the_shelf_pos;

            util::sys_clock_t timeout_clock;
            util::sys_clock_t IK_clock;

            double steering_time;
            double validation_time;
            double seeding_time;
            double planning_time;
            double steering_counter;
            double validation_counter;
            double planning_counter;

            // A variable to keep track whether we should load the roadmaps from a file or compute them
            bool load_roadmaps;
            bool has_roadmaps;


            // The current state of the manipulator as communicated from the robot
            sim::state_t* current_state;
            sim::state_t* multi_goal_assist;

            sim::plan_t IK_plan;
            sim::plan_t grasping_plan;
            sim::plan_t lifting_plan;
            sim::plan_t retracting_plan;
            sim::trajectory_t IK_traj;
            sim::trajectory_t grasping_traj;
            sim::trajectory_t lifting_traj;
            sim::trajectory_t retracting_traj;

            util::config_t retracting_config;
            
            // The information generated for a successful grasp
            std::string object_name;

            // The names of the degrees of freedom
            static const std::string dof_names[16];

            // The query to the task planner, also stores the plan
            base_apc_task_query_t* task_query;
            base_apc_task_query_t* lifting_task_query;
            base_apc_task_query_t* retracting_task_query;

            // Which hand is currently being controlled
            std::string _hand;
            // Whether the currently controlled hand is grasping something or not
            bool _grasping;
            // Names for the various motion planners used to control the robot
            // distinguished based on the hand and the manipulation mode 
            std::string left_grasped_planner, left_ungrasped_planner, right_grasped_planner, right_ungrasped_planner;
            // Names of different planning contexts where an object is active. 
            // There are 2 x num. objects such planning contexts. 
            // The multiplier of two comes from the two different hands.
            // Debugging variable
            std::vector<std::string> object_contexts;
            // A variable that allows us not to read from the callback the state of the robot
            // Useful during the planning process, when we want to have full control over the 
            // state of the robot
            bool dont_update_state;
            // A pointer to the state of the object
            // We read this over the network and is the output of the pose estimation process
            sim::state_t* object_start_state;

            // Pointers to the state space of the left manipulator and the right manipulator
            const util::space_t *left_manip_space, *right_manip_space;

            // A map from the names of the degrees of freedom to indices in the state spaces
            // of the left and the right manipulator
            util::hash_t<std::string, unsigned> index_map;

            // The pointers to the actual manipulator plants
            prx::packages::baxter::manipulator_plant_t *_left_manipulator, *_right_manipulator;
            
            // A hash table from names of obstacles to the actual systems
            util::hash_t<std::string, sim::system_ptr_t> obstacle_list;
            
            // A hash table from names of movable bodies to the actual systems
            util::hash_t<std::string, sim::system_ptr_t> object_list;
            
            // A hash table from object names to the known grasps for that object
            util::hash_t<std::string, std::vector< util::config_t > > robotiq_grasps;
            util::hash_t<std::string, std::vector< util::config_t > > unigripper_grasps;
            util::hash_t<std::string, std::vector< std::pair<util::config_t, int > > > right_grasps;
            util::hash_t<std::string, std::vector< std::pair<util::config_t, int > > > left_grasps;
            
            // A list of space points to use for grasp planning
            std::vector< util::space_point_t* > candidate_IKs;
            std::vector< util::space_point_t* > lifting_IKs;
            std::vector< util::space_point_t* > retraction_IKs;

            // A variable that keeps track whether Motoman has been activated or not
            // If false, we should first execute function "enable_robot"
            // and then try to execute a trajectory
            bool enabled_robot;
            
            //The IK data bases for the left and right arm
            bool store_IK_data_base;
            bool load_IK_data_base;

            IK_data_base_t* right_IK_data_base;
            IK_data_base_t* left_IK_data_base;
            IK_data_base_t* right_cam_IK_data_base;
            IK_data_base_t* left_cam_IK_data_base;

            std::string left_IK_file;
            std::string right_IK_file;
            
            bool store_trajectory_data;
            bool load_trajectory_data;

            bool using_robotiq_hand;

            bool visualize_grasps;

            std::vector< trajectory_db_entry > left_trajectories;
            std::vector< trajectory_db_entry > right_trajectories;

            std::vector< util::space_point_t* > IK_seeds;


            int grasp_id;
            int num_grasps_tried;
        };
    }
}

#endif
