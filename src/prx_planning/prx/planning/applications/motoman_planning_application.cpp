/**
 * @file base_apc_planning_application.cpp 
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

#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/communication/simulation_comm.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/goals/multiple_goal_states.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/applications/motoman_planning_application.hpp"
#include "prx/planning/modules/path_smoothers/path_smoother.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/simulation/sensing/sensors/point_cloud_sensor.hpp"
#include "prx/planning/task_planners/base_apc_task_planner/base_apc_task_planner.hpp"

#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "simulation/manipulator_simulator.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/communication/tf_broadcaster.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>
#include <yaml-cpp/yaml.h>
#include <boost/assign/list_of.hpp>

#include <algorithm>
#define ROBOT_SEND_COMMAND_TIMEOUT 0
#define ROBOT_AC_SERVER_TIMEOUT 5

#define LIFT_HEIGHT 0.02

#define GENERAL_IK 0
#define GRASPING_IK 1
#define LIFTING_IK 2
#define RETRACTION_IK 3

#define GRASPING_TIMEOUT 90

PLUGINLIB_EXPORT_CLASS(prx::plan::motoman_planning_application_t, prx::plan::planning_application_t)
namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace prx::packages::manipulation;
    namespace plan
    {
        using namespace comm;

        const std::string motoman_planning_application_t::dof_names[16] = {"torso_joint_b1", "arm_left_joint_1_s", "arm_left_joint_2_l", "arm_left_joint_3_e", "arm_left_joint_4_u", "arm_left_joint_5_r", "arm_left_joint_6_b", "arm_left_joint_7_t", "arm_right_joint_1_s", "arm_right_joint_2_l", "arm_right_joint_3_e", "arm_right_joint_4_u", "arm_right_joint_5_r", "arm_right_joint_6_b", "arm_right_joint_7_t", "torso_joint_b2"};

        motoman_planning_application_t::motoman_planning_application_t()
        {
            sem_init(&semaphore, 0, 1);
            _hand = "right";
            _grasping = false;
            _left_manipulator = NULL;
            _right_manipulator = NULL;
            left_IK_data_base = NULL;
            left_cam_IK_data_base = NULL;
            right_IK_data_base = NULL;
            right_cam_IK_data_base = NULL;
            enabled_robot = false;
            current_state = NULL;
            object_start_state = NULL;

            the_shelf_pos.resize(7);

            grasp_configurations.resize(1000);
        }

        motoman_planning_application_t::~motoman_planning_application_t()
        {
            model->get_state_space()->free_point(multi_goal_assist);
            model->get_state_space()->free_point(current_state); 
           
            for( unsigned i=0; i<candidate_IKs.size(); ++i )
                model->get_state_space()->free_point( candidate_IKs[i] );
            for( unsigned i=0; i<lifting_IKs.size(); ++i )
                model->get_state_space()->free_point( lifting_IKs[i] );
            for( unsigned i=0; i<retraction_IKs.size(); ++i )
                model->get_state_space()->free_point( retraction_IKs[i] );
            
            if( left_IK_data_base != NULL )
                delete left_IK_data_base;
            if( right_IK_data_base != NULL )
                delete right_IK_data_base;
            if( left_cam_IK_data_base != NULL )
                delete left_cam_IK_data_base;
            if( right_cam_IK_data_base != NULL )
                delete right_cam_IK_data_base;

            sem_destroy(&semaphore);
        }

        void motoman_planning_application_t::init(const parameter_reader_t* reader)
        {
            load_roadmaps = reader->get_attribute_as<bool>("load_roadmaps", true);
            has_roadmaps = false;
            if( load_roadmaps )
            {
                PRX_PRINT("This application will load roadmaps precomputed offline", PRX_TEXT_BLUE);
            }
            else
            {
                PRX_PRINT("This application will need to generate roadmaps online", PRX_TEXT_BLUE);
            }

            enabled_robot = false;
            std::string left = "left_";
            std::string right = "right_";
            std::string plugin_type_name = reader->get_attribute("plan_to_vis_comm", "visualization_comm");
            vis_comm = plan_base_communication_t::get_loader().createUnmanagedInstance("prx_planning/" + plugin_type_name);
            vis_comm->link_application(this);
            left_grasped_planner = "left_prm_grasp";
            left_ungrasped_planner = "left_prm";
            right_grasped_planner = "right_prm_grasp";
            right_ungrasped_planner = "right_prm";

            simulation::update_all_configs = false;
            //debug and communication flags
            debug = false;
            visualize = false;
            simulate = false;

            // initialize world model
            model = new world_model_t();
            reader->initialize(model, "world_model");
            model->use_context("full_space");
            std::vector< plant_t* > all_plants;
            model->get_system_graph().get_plants(all_plants);

            context_flags true_flags(true, true);
            context_flags active_flags(true, false);
            context_flags inactive_flags(false, false);
            util::hash_t<std::string, context_flags> mappings;

            foreach(plant_t* plant, all_plants)
            {
                if( _left_manipulator == NULL && dynamic_cast<manipulator_plant_t*>(plant) != NULL && plant->get_pathname().find(left) != std::string::npos )
                    _left_manipulator = static_cast<manipulator_plant_t*>(plant);
                if( _right_manipulator == NULL && dynamic_cast<manipulator_plant_t*>(plant) != NULL && plant->get_pathname().find(right) != std::string::npos )
                    _right_manipulator = static_cast<manipulator_plant_t*>(plant);
            }
            left_manip_space = _left_manipulator->get_state_space();
            right_manip_space = _right_manipulator->get_state_space();

            geom_map_t geom_map;
            model->get_simulator()->update_phys_geoms(geom_map);
            model->get_simulator()->update_obstacle_geoms(geom_map);
            tf_broadcasting();

            foreach(std::string name, geom_map | boost::adaptors::map_keys)
            {
                vis_comm->add_marker_to_array(geom_map[name], name);
            }
            vis_comm->publish_markers();
            tf_broadcasting();

            //left arm contexts
            mappings[_left_manipulator->get_pathname()] = true_flags;

            foreach(plant_t* plant, all_plants)
            {
                if( dynamic_cast<movable_body_plant_t*>(plant) != NULL )
                {
                    mappings[plant->get_pathname()] = active_flags;
                    std::string context_name = left + split_path(plant->get_pathname()).second;
                    // PRX_INFO_S("Setting up context for " << context_name);
                    object_contexts.push_back(context_name);
                    model->create_new_planning_context(context_name, mappings, inactive_flags);
                    mappings[plant->get_pathname()] = inactive_flags;
                }
            }
            //right arm contexts
            mappings.clear();
            mappings[_right_manipulator->get_pathname()] = true_flags;

            foreach(plant_t* plant, all_plants)
            {
                if( dynamic_cast<movable_body_plant_t*>(plant) != NULL )
                {
                    mappings[plant->get_pathname()] = active_flags;
                    std::string context_name = right + split_path(plant->get_pathname()).second;
                    std::string obj_name = split_path(plant->get_pathname()).second;
                    // PRX_PRINT("Set up the object mapping for: " << obj_name, PRX_TEXT_GREEN);
                    object_list[ obj_name ] = system_ptr_t( plant );
                    // PRX_INFO_S("Setting up context for " << context_name);
                    object_contexts.push_back(context_name);
                    model->create_new_planning_context(context_name, mappings, inactive_flags);
                    mappings[plant->get_pathname()] = inactive_flags;
                }
            }

            obstacle_list = model->get_obstacles();
            // ((comm::visualization_comm_t*)comm::vis_comm)->to_config = boost::bind(&world_model_t::get_configs, model, _1, _2);

            root_task = (task_planner_t*)reader->create_from_loader<planner_t > ("task_planner", "prx_planning");
            root_task->set_pathname("task_planner");
            parameters::initialize(root_task, reader, "task_planner", NULL, "");
            // reader->initialize(*root_task,"task_planner");
            root_task->link_world_model(model);

            if( reader->has_attribute("problems") )
            {
                parameter_reader_t::reader_map_t problem_reader = reader->get_map("problems");
                const parameter_reader_t* specification_template_reader = NULL;
                std::string specification_template_name;
                const parameter_reader_t* query_template_reader = NULL;
                std::string query_template_name;

                foreach(const parameter_reader_t::reader_map_t::value_type problem_item, problem_reader)
                {
                    if( problem_item.second->has_attribute("specification/template") )
                    {
                        specification_template_name = problem_item.second->get_attribute("specification/template");
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + specification_template_name);
                    }

                    specification_t* specification = parameters::initialize_from_loader<specification_t>("prx_planning", problem_item.second, "specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }

                    root_specifications.push_back(specification);
                    if( problem_item.second->has_attribute("query") )
                    {
                        PRX_DEBUG_COLOR("Found query with specification", PRX_TEXT_BLUE);
                        if( problem_item.second->has_attribute("query/template") )
                        {
                            query_template_name = problem_item.second->get_attribute("query/template");
                            query_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + query_template_name);
                        }

                        query_t* query = parameters::initialize_from_loader<query_t>("prx_planning", problem_item.second, "query", query_template_reader, "");
                        root_queries.push_back(query);
                        task_query = dynamic_cast<base_apc_task_query_t*>(query);

                        query_t* query_2 = parameters::initialize_from_loader<query_t>("prx_planning", problem_item.second, "query", query_template_reader, "");
                        lifting_task_query = dynamic_cast<base_apc_task_query_t*>(query_2);

                        query_t* query_3 = parameters::initialize_from_loader<query_t>("prx_planning", problem_item.second, "query", query_template_reader, "");
                        retracting_task_query = dynamic_cast<base_apc_task_query_t*>(query_3);

                        if( query_template_reader != NULL )
                        {
                            delete query_template_reader;
                            query_template_reader = NULL;
                        }
                    }
                    else
                    {
                        PRX_FATAL_S("No intial query found in input. Please add a query");
                    }
                }
            }
            else
            {
                PRX_FATAL_S("No problem definition read from input!");
            }

            stop_updating_state();
            sub_joint = n.subscribe("/joint_states", 1, &motoman_planning_application_t::get_state_callback, this);
            vis_array_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
            cancel_publisher = n.advertise<actionlib_msgs::GoalID>("/joint_trajectory_action/cancel", 1);

            current_state = model->get_state_space()->alloc_point();
            multi_goal_assist = model->get_state_space()->alloc_point();
            
            unsigned num_ik_seeds = reader->get_attribute_as<double>("num_ik_seeds", 70);
            for( unsigned i=0; i<num_ik_seeds; ++i )
            {
                candidate_IKs.push_back( model->get_state_space()->alloc_point() );
                lifting_IKs.push_back( model->get_state_space()->alloc_point() );
                retraction_IKs.push_back( model->get_state_space()->alloc_point() );
            }

            IK_seeds.resize( num_ik_seeds );
            for( int i=0; i<IK_seeds.size(); i++ )
                IK_seeds[i] = NULL;
            
            load_grasp_data( reader );

            store_IK_data_base = reader->get_attribute_as<bool>("store_IK_data_base", false);
            load_IK_data_base = reader->get_attribute_as<bool>("load_IK_data_base", false);

            store_trajectory_data = reader->get_attribute_as<bool>("store_trajectory_data_base", false);
            load_trajectory_data = reader->get_attribute_as<bool>("load_trajectory_data_base", false);

            visualize_grasps = reader->get_attribute_as<bool>("visualize_grasps", false);

            if( store_trajectory_data )
            {
                PRX_PRINT("Application will STORE trajectory information.", PRX_TEXT_CYAN);
            }
            if( load_trajectory_data )
            {
                PRX_PRINT("Application will LOAD trajectory information.", PRX_TEXT_GREEN);
            }

            left_IK_data_base = NULL;
            right_IK_data_base = NULL;
            if(store_IK_data_base || load_IK_data_base)
            {
                if(reader->has_attribute("left_IK_data_base"))
                {
                    left_IK_data_base = new IK_data_base_t();
                    left_IK_data_base->init(reader->get_child("left_IK_data_base").get());
                }
                else
                    PRX_FATAL_S("You requested to store/load the IK data bases but you have not initialize the left data base!");

                left_IK_file = reader->get_attribute("left_IK_file", "left_IK_data_base.txt");

                if(reader->has_attribute("left_cam_IK_data_base"))
                {
                    left_cam_IK_data_base = new IK_data_base_t();
                    left_cam_IK_data_base->init(reader->get_child("left_cam_IK_data_base").get());
                }
                else
                    PRX_FATAL_S("You requested to store/load the IK data bases but you have not initialize the left camera data base!");

                if(reader->has_attribute("right_IK_data_base"))
                {
                    right_IK_data_base = new IK_data_base_t();
                    right_IK_data_base->init(reader->get_child("right_IK_data_base").get());
                }
                else
                    PRX_FATAL_S("You requested to store/load the IK data bases but you have not initialize the right data base!");

                right_IK_file = reader->get_attribute("right_IK_file", "right_IK_data_base.txt");

                if(reader->has_attribute("right_cam_IK_data_base"))
                {
                    right_cam_IK_data_base = new IK_data_base_t();
                    right_cam_IK_data_base->init(reader->get_child("right_cam_IK_data_base").get());
                }
                else
                    PRX_FATAL_S("You requested to store/load the IK data bases but you have not initialize the right camera data base!");

            }

            task_query->get_goal()->link_space( model->get_state_space() );
            lifting_task_query->get_goal()->link_space( model->get_state_space() );
            retracting_task_query->get_goal()->link_space( model->get_state_space() );

            if( load_trajectory_data )
            {
                PRX_PRINT("\n\n LOADING TRAJECTORY DATA \n\n", PRX_TEXT_CYAN);
                load_trajectories();
            }

            if( load_roadmaps && !has_roadmaps )
            {
                PRX_PRINT("\n\n\n COMPUTE ROADMAPS \n\n\n", PRX_TEXT_LIGHTGRAY);
                compute_roadmaps();
            }
        }

        void motoman_planning_application_t::execute()
        {
           start_updating_state();
            for( int i = 0; i < 15; i++ )
                index_map[ dof_names[i] ] = i;

            // get the execution things
            for( int i = 1; i < 15; i++ )
                motoman_command.trajectory.joint_names.push_back(dof_names[i]);
            motoman_command.trajectory.joint_names.push_back(dof_names[0]);
            motoman_command.trajectory.joint_names.push_back(dof_names[15]);
            unigripper_command.trajectory.joint_names.push_back("head_hinge");

            model->use_context(_hand + "_manipulator");

            // setting up callbacks
            clear_objects_service = n.advertiseService("clear_objects", &motoman_planning_application_t::clear_objects_callback, this);
            clear_object_service = n.advertiseService("clear_object", &motoman_planning_application_t::clear_object_callback, this);
            place_object_service = n.advertiseService("place_object", &motoman_planning_application_t::place_object_callback, this);
            grasp_service = n.advertiseService("grasp", &motoman_planning_application_t::grasp_callback, this);
            release_service = n.advertiseService("release", &motoman_planning_application_t::release_callback, this);
            validate_end_effector_service = n.advertiseService("validate_end_effector", &motoman_planning_application_t::validate_end_effector_callback, this);
            compute_grasp_service = n.advertiseService("compute_grasp", &motoman_planning_application_t::compute_grasp_callback, this);

            // boost::bind(&distance_function_t::distance, this, _1, _2);
            compute_plan_server = new CServer(n, "compute_plan", boost::bind(&motoman_planning_application_t::compute_plan_server_callback, this, _1), false);
            execute_plan_server = new EServer(n, "execute_plan", boost::bind(&motoman_planning_application_t::execute_plan_server_callback, this, _1), false);
            compute_plan_server->start();
            execute_plan_server->start();
            ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("joint_trajectory_action", false);
            ac_uni = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("unigripper/unigripper_joint_trajectory_action", false);

            PRX_PRINT("\n\n prx_planning is ready to receive queries. \n\n ", PRX_TEXT_GREEN);
        }

        void motoman_planning_application_t::get_state_callback(const sensor_msgs::JointState& stateMsg)
        {
            int value; 
            sem_getvalue(&semaphore, &value); 
            if( value==0 || !has_roadmaps )
                return;
            stop_updating_state();

            sensor_msgs::JointState start_state = stateMsg;
            for( unsigned i = 0; i < start_state.name.size(); i++ )
            {
                if( index_map.find(start_state.name[i]) != index_map.end() )
                    current_state->at(index_map[start_state.name[i]]) = start_state.position[i];
                else if( start_state.name[i] == "head_hinge" )
                    current_state->at(15) = start_state.position[i];
            }
            left_manip_space->copy_from_point(current_state);
            right_manip_space->copy_from_point(current_state);

            if( _grasping )
            {
                current_state->at(16) = 1;
                ((manipulation_simulator_t*)model->get_simulator())->update_ground_truth();
            }
            else
                current_state->at(16) = 0;
            start_updating_state();

            tf_broadcasting();
        }

        void motoman_planning_application_t::compute_plan_server_callback(const prx_planning::compute_planGoalConstPtr& req)
        {
            // PRX_INFO_S( "===== COMPUTE PLAN @ state  ======================" );
 
            //if( _grasping )
            //PRX_PRINT( "object state at compute plan: " << model->get_active_space()->print_memory( 7 ), PRX_TEXT_GREEN );

            if( !has_roadmaps && !load_roadmaps )
            {
                stop_updating_state();
                compute_roadmaps();
                start_updating_state();
            }
            if( req->dest_type == "origin" )
            {
                place_albertos_box( true );
                ((base_apc_task_planner_t*)(this->root_task))->restart_planners();
            }
            sleep(1);

            // check to see if the planning context needs to be changed depending on the arm
            check_for_context_switch(req->arm);

            // setting up the goal of the query
            sensor_msgs::JointState end_state = req->end_state;       
            ((base_apc_task_query_t*)task_query)->clear_goal();
            ((multiple_goal_states_t*)task_query->get_goal())->add_goal_state( current_state );

            for( unsigned i = 0; i < end_state.name.size(); i++ )
            {
                if( index_map.find(end_state.name[i]) != index_map.end() )
                    task_query->get_goal()->get_goal_points()[0]->at(index_map[end_state.name[i]]) = end_state.position[i];
                else if( end_state.name[i] == "head_hinge" )
                    task_query->get_goal()->get_goal_points()[0]->at(15) = end_state.position[i];            
            }

            // PRX_PRINT( " Initial state :: " <<  model->get_state_space()->print_point( current_state, 3 ) , PRX_TEXT_LIGHTGRAY );
            // PRX_PRINT( " Goal state :: " <<  model->get_state_space()->print_point( task_query->get_goal()->get_goal_points()[0], 3 ), PRX_TEXT_LIGHTGRAY );

            // computing the actual path
            stop_updating_state();
            bool success = find_path_internal( current_state, task_query->get_goal()->get_goal_points()[0], GENERAL_IK );
            start_updating_state();

            // PRX_INFO_S( "Do we have a path? " << success );
            if( success )
            {
                // PRX_INFO_S( "Success in computing plan" );
                compute_plan_server->setSucceeded();
            }
            else
            {
                // PRX_INFO_S( "Failure in computing plan" );
                compute_plan_server->setAborted();
            }

            //if( _grasping )
            //    PRX_PRINT( "object state at compute plan's end: " << model->get_active_space()->print_memory( 3 ), PRX_TEXT_GREEN );
        }

        void motoman_planning_application_t::execute_plan_server_callback(const prx_planning::execute_planGoalConstPtr& req)
        {
            // PRX_INFO_S( "==================   EXECUTE PLAN   ======================");
            // PRX_PRINT( "initial state is: " << model->get_state_space()->print_point( current_state, 3 ), PRX_TEXT_GREEN );
            //if( _grasping )
            //PRX_PRINT( "object state at execute plan's end: " << model->get_active_space()->print_memory( 3 ), PRX_TEXT_GREEN );
            //PRX_PRINT( "context name is: " << model->get_current_context(), PRX_TEXT_GREEN );

            // read the speed with which the arm should move
            double speed = req->speed;
            if( speed < .1 || speed > 1 )
            {
                PRX_WARN_S("Speed multiplier for motoman execution is out of range [.1,1]. Ts ts ts ts. Setting to .2");
                speed = .2;
            }

            // make sure the robot is enabled
            if( !enabled_robot )
            {
                state_t* state = task_query->get_start_state();
                trajectory_t traj;
                traj.link_space(model->get_state_space());
                traj.copy_onto_back(state);
                traj.copy_onto_back(state);
                communicate_trajectory(traj, speed);
            }
            
            manipulator_plant_t* manipulator = NULL;
            if( _hand == "left" )
                manipulator = _left_manipulator;
            else
                manipulator = _right_manipulator;

            plan_t prop_plan;
            trajectory_t compare_traj;
            space_point_t* start = NULL;
            space_point_t* goal = NULL;
            config_t ee_config;

            if(req->traj_type=="query")
            {
                prop_plan = task_query->plan;
                compare_traj = task_query->path;
            }
            else if(req->traj_type=="IK")
            {
                prop_plan = IK_plan;
                compare_traj = IK_traj;
                                
                if( compare_traj.size() > 0 )
                {
                    //PRX_PRINT("Setting start/goal pair for IK type.", PRX_TEXT_MAGENTA);
                    start = compare_traj.at( 0 );
                    goal = compare_traj.back();
                    model->push_state( goal );
                    manipulator->get_end_effector_configuration( ee_config );
                    //PRX_PRINT("The end effector is: " << ee_config, PRX_TEXT_MAGENTA);
                }
            }
            else if(req->traj_type=="grasping")
            {
                prop_plan = grasping_plan;
                compare_traj = grasping_traj;
            }
            else if(req->traj_type=="lifting")
            {
                prop_plan = lifting_plan;
                compare_traj = lifting_traj;
            }
            else if(req->traj_type=="retracting")
            {
                prop_plan = retracting_plan;
                compare_traj = retracting_traj;
            }

            /*
            trajectory_t prop_traj(model->get_state_space());
            // propagate the plan to rediscover the trajectory to be followed
            stop_updating_state();
            //PRX_INFO_S("Before propagating plan... grasping is " << _grasping );            
            //PRX_PRINT( "The object before: " << model->get_active_space()->print_memory( ), PRX_TEXT_BROWN );
            model->propagate_plan( current_state, prop_plan, prop_traj, true );
            //PRX_PRINT( "The object after: " << model->get_active_space()->print_memory( ), PRX_TEXT_BROWN );
            if(prop_traj.in_collision() || (prop_traj.size() > 1 && prop_traj.size() != compare_traj.size() ) )
            {
                if( prop_traj.size() != compare_traj.size() )
                {
                    PRX_ERROR_S( "The two trajectories are of different size" );
                }
                if( prop_traj.in_collision() )
                {
                    PRX_ERROR_S("The computed trajectory is in collision. Bad juju.");
                }

                PRX_INFO_S("Stored: \n"<<compare_traj.print());
                PRX_INFO_S("Computed: \n"<<prop_traj.print());

                if(!_grasping)
                {
                    for( int i=0; i<prop_traj.size(); i++ )
                    {
                        model->push_state( prop_traj[i] );
                        collision_list_t* list = model->get_colliding_bodies();
                        if( list->size() > 0 )
                        {
                            PRX_WARN_S("failed index is: " << i );
                            foreach(collision_pair_t pair, list->get_body_pairs())
                            {
                                PRX_WARN_S(pair.first << " " << pair.second);
                            }
                            PRX_WARN_S("");

                            config_t ee_pose;
                            manipulator->get_end_effector_configuration( ee_pose );
                            PRX_WARN_S("EE config: " << ee_config );
                        }
                    }
                }
                execute_plan_server->setAborted();
                start_updating_state();

                if( prop_traj.size() != compare_traj.size() )
                {
                    PRX_FATAL_S( "The two trajectories are of different size" );
                }
                if( prop_traj.in_collision() )
                {
                    PRX_FATAL_S("The computed trajectory is in collision. Bad juju.");
                }

                return;
            }
            //PRX_PRINT( "The object after: " << model->get_active_space()->print_memory( 3 ), PRX_TEXT_BROWN );
            //PRX_INFO_S("After propagating plan... of length " << task_query->plan.length() );
            start_updating_state();

            // if there is a plan to execute
            if( prop_plan.length() > 0 )
            */
            if( compare_traj.size() > 1 )
            {
                // take the plan in task_query and publish to JointCommand
                bool suc = communicate_trajectory(compare_traj, speed);
                // PRX_PRINT("Finished communicating trajectory with success: "<<suc,PRX_TEXT_GREEN);

                if( suc )
                {
                    // if we are grasping an object, make sure we set the object to be at the end of the executed plan
                    sleep(1);
                    execute_plan_server->setSucceeded();
                    
                    // Since we successfully transmitted this trajectory, store it in the database
                    if( store_trajectory_data && start != NULL )
                    {
                        // PRX_PRINT("Adding a trajectory to the database!", PRX_TEXT_MAGENTA);
                        add_trajectory_to_database( start, ee_config, compare_traj );
                    }
                }
                else
                {
                    //DEBUG: Breaking when we fail to communicate the trajectory.
                    // PRX_WARN_S("Somehow got a failure back from the Robot. Previously, this would stop everything!");
                    execute_plan_server->setAborted();
                }
            }
            else
                execute_plan_server->setSucceeded();

            //if( _grasping )
            //    PRX_PRINT( "object state at execute plan's end: " << model->get_active_space()->print_memory( 7 ), PRX_TEXT_GREEN );

           // PRX_PRINT( "resulting state is: " << model->get_state_space()->print_point( current_state, 3 ), PRX_TEXT_GREEN );
        }

        bool motoman_planning_application_t::place_object_callback(prx_planning::place_object::Request& req, prx_planning::place_object::Response& res)
        {
            std::string context_name = _hand + "_" + req.object_name;
            std::string obstacle_name = "simulator/obstacles/" + req.object_name;
            geometry_msgs::PoseStamped pose = req.object_pose;

            // If the input object is in the obstacle list, then move it to the input pose
            if( obstacle_list.find(obstacle_name) != obstacle_list.end() )
            {
                // PRX_INFO_S("Moving " << req.object_name << " obstacle in planning node.");
                config_t obs_config;
                obs_config.set_position(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                obs_config.set_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
                static_cast<obstacle_t*>(obstacle_list[obstacle_name].get())->update_root_configuration(obs_config);
                model->get_simulator()->update_obstacles_in_collision_checker();
                std::string bin_name, remaining;
                boost::tie( bin_name, remaining ) = split_path_delimiter( req.object_name, '_');
                if( req.object_name == "shelf" || bin_name == "bin" || bin_name =="albertos" )
                {
                    the_shelf_pos[0] = pose.pose.position.x;
                    the_shelf_pos[1] = pose.pose.position.y;
                    the_shelf_pos[2] = pose.pose.position.z;
                    the_shelf_pos[3] = pose.pose.orientation.x;
                    the_shelf_pos[4] = pose.pose.orientation.y;
                    the_shelf_pos[5] = pose.pose.orientation.z;
                    the_shelf_pos[6] = pose.pose.orientation.w;
                    this->root_task->set_param("", "shelf_position", the_shelf_pos);
                }
                // PRX_INFO_S("Done moving " << req.object_name << " obstacle in planning node.");
            }
            // Otherwise, we need to activate the object and move it to the input pose
            else
            {
                // PRX_INFO_S("Adding " << req.object_name << " to the planning node: " << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z);
                model->use_context(context_name);
                if( object_start_state == NULL )
                    object_start_state = model->get_active_space()->alloc_point();
                object_start_state->at(0) = pose.pose.position.x;
                object_start_state->at(1) = pose.pose.position.y;
                object_start_state->at(2) = pose.pose.position.z;
                object_start_state->at(3) = pose.pose.orientation.x;
                object_start_state->at(4) = pose.pose.orientation.y;
                object_start_state->at(5) = pose.pose.orientation.z;
                object_start_state->at(6) = pose.pose.orientation.w;
                model->get_active_space()->copy_from_point(object_start_state);
                
                sim::update_point_cloud = true;
                model->update_sensing();
                sim::update_point_cloud = false;

                // PRX_INFO_S("Done adding " << context_name << " to the planning node: ");
            }
            res.success = true;
        }

        void motoman_planning_application_t::place_albertos_box( bool activate )
        {
            std::string obstacle_name = "simulator/obstacles/albertos_box";

            config_t obs_config;           
            if( activate)
            {
                //PRX_PRINT( "\n\n\nThe shelf pose is: " << the_shelf_pos[0] << " " << the_shelf_pos[1] << " " <<the_shelf_pos[2] << " " <<the_shelf_pos[3] << " " <<the_shelf_pos[4] << " " <<the_shelf_pos[5] << " " <<the_shelf_pos[6] << "\n\n\n", PRX_TEXT_BROWN );

                obs_config.set_position( the_shelf_pos[0], the_shelf_pos[1], the_shelf_pos[2]);
                obs_config.set_orientation( the_shelf_pos[3], the_shelf_pos[4], the_shelf_pos[5], the_shelf_pos[6] );
            }
            else
            {
                obs_config.set_position( -1999, -1999, -1999 );
                obs_config.set_orientation( 0, 0, 0, 1 );
            }
                
            static_cast<obstacle_t*>(obstacle_list[obstacle_name].get())->update_root_configuration(obs_config);

            model->get_simulator()->update_obstacles_in_collision_checker();
        }

        bool motoman_planning_application_t::clear_objects_callback(prx_planning::clear_objects::Request& req, prx_planning::clear_objects::Response& res)
        {
            //Define a configuration far far away
            config_t obs_config;
            obs_config.set_position(-999, -999, -999);
            obs_config.set_orientation(0, 0, 0, 1);

            foreach(std::string name, obstacle_list | boost::adaptors::map_keys)
                static_cast<obstacle_t*>(obstacle_list[name].get())->update_root_configuration(obs_config);
            model->get_simulator()->update_obstacles_in_collision_checker();

            if( _hand == "left" )
                model->use_context("left_manipulator");
            else
                model->use_context("right_manipulator");
            // tf_broadcasting();
            res.success = true;
            return true;
        }

        bool motoman_planning_application_t::clear_object_callback(prx_planning::clear_object::Request& req, prx_planning::clear_object::Response& res)
        {
            std::string context_name = _hand + "_" + req.object_name;
            std::string obstacle_name = "simulator/obstacles/" + req.object_name;

            // If the input object is in the obstacle list, then move it far far away
            if( obstacle_list.find(obstacle_name) != obstacle_list.end() )
            {
                // PRX_INFO_S("Removing " << req.object_name << " obstacle in planning node.");
                config_t obs_config;
                obs_config.set_position(-999, -999, -999);
                obs_config.set_orientation(0, 0, 0, 1);
                static_cast<obstacle_t*>(obstacle_list[obstacle_name].get())->update_root_configuration(obs_config);
                model->get_simulator()->update_obstacles_in_collision_checker();
                //PRX_INFO_S("Done Removing " << req.object_name << " obstacle in planning node.");
            }
                // Otherwise, change the planning context so that it is not active anymore
            else
            {
                // PRX_INFO_S("Removing " << req.object_name << " in planning node.");
                std::string old_context = model->get_current_context();
                if( old_context == context_name )
                {
                    if( _hand == "left" )
                        model->use_context("left_manipulator");
                    else
                        model->use_context("right_manipulator");
                }
                std::vector<double> clear_state = boost::assign::list_of(-999)(-999)(-999)(0)(0)(0)(1);
                foreach(system_ptr_t ptr, object_list | boost::adaptors::map_values)
                {
                    ptr->get_state_space()->set_from_vector(clear_state);
                }
                //PRX_INFO_S("Done removing " << context_name << " in planning node.");
            }
            // tf_broadcasting();
            res.success = true;
            return true;
        }

        bool motoman_planning_application_t::grasp_callback(prx_planning::grasp::Request& req, prx_planning::grasp::Response& res)
        {
            // PRX_INFO_S( "==================  GRASP CALLBACK  ===== for object " << req.object_name);
            model->use_context(_hand + "_" + req.object_name);
            _grasping = true;
            res.success = true;

            stop_updating_state();
            if( _grasping )
                current_state->at(16) = 1;

            model->push_state( current_state );
            ((manipulation_simulator_t*)model->get_simulator())->compute_relative_config();
            start_updating_state();

            //PRX_PRINT( "current object state at grasping: " << model->get_active_space()->print_memory( 3 ), PRX_TEXT_GREEN );
            return true;
        }

        bool motoman_planning_application_t::release_callback(prx_planning::release::Request& req, prx_planning::release::Response& res)
        {
            // PRX_INFO_S( "==================  RELEASE CALLBACK  ===== " );

            stop_updating_state();
            model->push_state( current_state );

            //PRX_PRINT( "current object state at releasing: " << model->get_active_space()->print_memory( 3 ), PRX_TEXT_GREEN );

            model->use_context(_hand + "_" + "manipulator");

            _grasping = false;
            res.success = true;

            if( _grasping )
                current_state->at(16) = 0;

            model->push_state( current_state );  
            ((manipulation_simulator_t*)model->get_simulator())->compute_relative_config();
          
            start_updating_state();

            return true;
        }

        bool motoman_planning_application_t::validate_end_effector_callback(prx_planning::validate_end_effector::Request& req, prx_planning::validate_end_effector::Response& res)
        {
            // PRX_INFO_S( "==================  VALIDATE END EFFECTOR  ======================");
            //if( _grasping )
            //    PRX_PRINT( "current object state at start of validation: " << model->get_active_space()->print_memory( 7 ), PRX_TEXT_GREEN );

            // check to see if we need to switch contexts
            check_for_context_switch(req.arm);

            manipulator_plant_t* manipulator = NULL;
            if( _hand == "left" )
                manipulator = _left_manipulator;
            else
                manipulator = _right_manipulator;
            manipulator->get_state_space()->copy_from_point(current_state);

            // Getting the input pose into a configuration
            config_t config;
            geometry_msgs::PoseStamped pose = req.pose;
            config.set_position(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z-.01);
            config.set_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

            // Arguments are: camera or end effector, current state, target config, returned state, reachable plan, type of IK solution            

            sys_clock_t zak_clock;
            zak_clock.reset();

            timeout_clock.reset();
            stop_updating_state();

            bool success = check_for_stored_trajectory( current_state, config );
            if( success == false )
                success = reachable_IK( req.camera, current_state, config, GENERAL_IK );

            start_updating_state();
            // PRX_PRINT("overall_time " << zak_clock.measure(), PRX_TEXT_BROWN );

            // if successful, drop an egg
            if( success )
            {
                std::vector<double> params;
                params.push_back(.02);
                geometry_t geom(PRX_SPHERE,&params);
                
                vis_array_pub.publish(vis_comm->send_marker(geom, "EE_config", config));
            }
            // PRX_INFO_S("Valid IK: " << success);
            res.success = success;
            if (res.success)
            {
                int robot_state_size = IK_traj.back()->memory.size();
                res.arm_config.position.resize(robot_state_size);
                res.arm_config.name.resize(robot_state_size);
                for( unsigned i = 0; i < 16; i++ )
                {
                    res.arm_config.name[i] = dof_names[i];
                    res.arm_config.position[i] = IK_traj.back()->at(index_map[dof_names[i]]);
                }
                res.arm_config.position[16] = IK_traj.back()->at(15);
                res.arm_config.name[16] = "head_hinge";

//                int robot_state_size = IK_traj.back()->memory.size();
//                res.arm_config.position.resize(robot_state_size);
//                res.arm_config.name.resize(robot_state_size);
//                for (int i = 1; i < 15; i++)
//                {
//                    res.arm_config.position[i - 1] = IK_traj.back()->at(i);
//                    res.arm_config.name[i - 1] = dof_names[i];
//                }
//                res.arm_config.position[14] = IK_traj.back()->at(0);
//                res.arm_config.position[15] = IK_traj.back()->at(0);
//                res.arm_config.position[16] = IK_traj.back()->at(15);
//                res.arm_config.name[14] = dof_names[0];
//                res.arm_config.name[15] = dof_names[15];
//                res.arm_config.name[16] = "head_hinge";
            }

            //if( _grasping )
            //    PRX_PRINT( "current object state at end of validation: " << model->get_active_space()->print_memory( 7 ), PRX_TEXT_GREEN );

            place_albertos_box( false );

            return true;
        }

        void motoman_planning_application_t::tf_broadcasting()
        {
            bool flag = false;
            if( !dont_update_state )
            {
                flag = true;
                stop_updating_state();
            }
            PRX_ASSERT(tf_broadcaster != NULL);

            config_list_t config_map;
            unsigned index = 0;
            // Include in the config_map the configurations of all active plants and objects
            model->get_simulator()->update_phys_configs(config_map, index);
            // Make sure you include both manipulators (redundancy here for the active manipulator)
            _left_manipulator->update_phys_configs(config_map, index);
            _right_manipulator->update_phys_configs(config_map, index);
            // Include all the obstacles
            model->get_simulator()->update_obstacles_configs(config_map, index);

            // transmit configurations
            for( config_list_t::iterator iter = config_map.begin(); iter != config_map.end(); iter++ )
                tf_broadcaster->queue_config(iter->second, iter->first);
            tf_broadcaster->broadcast_configs();
            if( flag )
               start_updating_state();
        }

        bool motoman_planning_application_t::communicate_trajectory(trajectory_t& traj, double speed)
        {
            motoman_command.trajectory.points.clear();
            unigripper_command.trajectory.points.clear();

            //PRX_PRINT("Communicating trajectory of size: " << traj.size() << " Speed: " << speed, PRX_TEXT_BLUE);
            double duration = 0;
            if( traj.size() == 0 )
                return true;
            double sim_step = simulation::simulation_step;
            if(traj.size()==2)
                sim_step = .5;

            foreach(state_t* state, traj)
            {
                //setup the JointTrajectoryPoint
                trajectory_msgs::JointTrajectoryPoint point;
                point.time_from_start = ros::Duration(duration);
                trajectory_msgs::JointTrajectoryPoint point_uni;
                point_uni.time_from_start = ros::Duration(duration);
                for( unsigned i = 1; i < 15; i++ )
                {
                    point.positions.push_back(state->memory[i]);
                    point.velocities.push_back(0);
                    point.accelerations.push_back(0);
                }
                point.positions.push_back(state->memory[0]);
                point.velocities.push_back(0);
                point.accelerations.push_back(0);
                point.positions.push_back(state->memory[0]);
                point.velocities.push_back(0);
                point.accelerations.push_back(0);

                point_uni.positions.push_back(state->memory[15]);
                point_uni.velocities.push_back(0);
                point_uni.accelerations.push_back(0);

                duration += ((int)(1.0 / speed)) * sim_step;
                motoman_command.trajectory.points.push_back(point);
                unigripper_command.trajectory.points.push_back(point_uni);
            }
            double duration_of_timeout = (traj.size()*((int)(1.0 / speed)) * sim_step)*1.5+1.5;
            motoman_command.trajectory.header.stamp = ros::Time::now();
            unigripper_command.trajectory.header.stamp = ros::Time::now();
            //PRX_PRINT("Motoman Command Size: " << motoman_command.trajectory.points.size() << " and Time Stamp: " << motoman_command.trajectory.header.stamp, PRX_TEXT_BLUE);

            //PRX_PRINT("Wait for Server", PRX_TEXT_BLUE);
            bool found_ac_server = false;
            while( !found_ac_server )
                found_ac_server = ac->waitForServer(ros::Duration(ROBOT_AC_SERVER_TIMEOUT)) && ac_uni->waitForServer(ros::Duration(ROBOT_AC_SERVER_TIMEOUT));
            // sleep(2);

            //PRX_PRINT("Ready to Send Goal", PRX_TEXT_BLUE);
            ac->sendGoal(motoman_command);
            ac_uni->sendGoal(unigripper_command);

            bool finished_before_timeout;
            finished_before_timeout = ac->waitForResult(ros::Duration(duration_of_timeout));

            //PRX_PRINT("Done waiting...", PRX_TEXT_BLUE);
            enabled_robot = true;

            actionlib::SimpleClientGoalState state = ac->getState();
            // PRX_INFO_S("AC state: " << state.toString());

            // sys_clock_t clock;
            // clock.reset();
            // while(clock.measure()<3 && state.toString() != "SUCCEEDED")
            // { 
            //     state = ac->getState();
            //     PRX_INFO_S("AC state: " << state.toString());
            // }

            if( !finished_before_timeout )
            {

                actionlib_msgs::GoalID empty_msg;
                cancel_publisher.publish(empty_msg);
                bool ret_value = false;
                // PRX_INFO_S("Time of " << duration_of_timeout << "s met. Abort.");
                sleep(1);
                stop_updating_state();
                if(model->get_state_space()->distance(current_state,traj.back()) < .01  )
                    ret_value = true;
                start_updating_state();

                //PRX_INFO_S("AC state: " << state.toString());
                return ret_value;
            }
            else if( state.toString() == "SUCCEEDED" )
                    return true;
            else
                return false;
        }

        void motoman_planning_application_t::compute_roadmaps()
        {
            PRX_PRINT("compute roadmaps\n",PRX_TEXT_BROWN);
                
            if(load_IK_data_base)
            {
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_output/IK_data_bases/");
                
                // LEFT
                std::string file = dir + left_IK_file;
                PRX_PRINT("Load left IK data base from the file: " << file, PRX_TEXT_CYAN);
                
                std::ifstream fin_left;
                fin_left.open(file.c_str());
                
                if(!fin_left.is_open())
                    PRX_FATAL_S("Could not open the file : " << file);
                left_IK_data_base->deserialize(fin_left, _left_manipulator->get_state_space());
                
                fin_left.close();
                fin_left.clear();
                
                file = dir + "left_camera.database";
                fin_left.open(file.c_str());
                
                if(!fin_left.is_open())
                    PRX_FATAL_S("Could not open the file : " << file);
                left_cam_IK_data_base->deserialize(fin_left, _left_manipulator->get_state_space());
                
                // RIGHT
                file = dir + right_IK_file;
                PRX_PRINT("Load right IK data base from the file: " << file, PRX_TEXT_CYAN);
                
                std::ifstream fin_right;
                fin_right.open(file.c_str());
                
                if(!fin_right.is_open())
                    PRX_FATAL_S("Could not open the file : " << file);
                right_IK_data_base->deserialize(fin_right, _right_manipulator->get_state_space());
                
                fin_right.close();
                fin_right.clear();
                
                file = dir + "right_camera.database";
                fin_right.open(file.c_str());
                
                if(!fin_right.is_open())
                    PRX_FATAL_S("Could not open the file : " << file);
                right_cam_IK_data_base->deserialize(fin_right, _right_manipulator->get_state_space());
            }
            
            std::string old_context = model->get_current_context();
            this->root_task->link_specification(root_specifications[0]);
            dynamic_cast<base_apc_task_planner_t*>(this->root_task)->link_manipulators(_left_manipulator, _right_manipulator);
            dynamic_cast<base_apc_task_planner_t*>(this->root_task)->link_IK_data_base(left_IK_data_base, right_IK_data_base, left_cam_IK_data_base, right_IK_data_base);
            this->root_task->setup();
            
            if( load_roadmaps )
            {
                model->use_context("left_manipulator");
                ((base_apc_task_planner_t*)root_task)->deserialize("left_prm");
                ((base_apc_task_planner_t*)root_task)->special_deserialize("left_prm_grasp");
                model->use_context("right_manipulator");
                ((base_apc_task_planner_t*)root_task)->deserialize("right_prm");
                ((base_apc_task_planner_t*)root_task)->special_deserialize("right_prm_grasp");
            }
            else
            {
                this->root_task->execute();
                place_albertos_box( true );                    
                ((base_apc_task_planner_t*)(this->root_task))->special_serialize();
                place_albertos_box( false );
            }
            
            if( store_IK_data_base )
            {
                //Clear out any previous information we have
                left_IK_data_base->clear();
                right_IK_data_base->clear();
                left_cam_IK_data_base->clear();
                right_cam_IK_data_base->clear();
                
                PRX_PRINT("Going to store IK data bases", PRX_TEXT_CYAN);
                dynamic_cast<base_apc_task_planner_t*>(this->root_task)->generate_IK_data_base(*left_IK_data_base, *right_IK_data_base, *left_cam_IK_data_base, *right_cam_IK_data_base);
                
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                dir += ("/prx_output/");
                boost::filesystem::path output_dir(dir);
                if( !boost::filesystem::exists(output_dir) )
                {
                    boost::filesystem::create_directory(output_dir);
                }
                dir += ("IK_data_bases/");
                boost::filesystem::path output_dir2(dir);
                if( !boost::filesystem::exists(output_dir2) )
                {
                    boost::filesystem::create_directory(output_dir2);
                }
                
                // LEFT
                std::string file = dir + left_IK_file;
                PRX_PRINT("Store left IK data base at the file: " << file, PRX_TEXT_CYAN);
                
                std::ofstream fout_left;
                fout_left.open(file.c_str());
                
                if(!fout_left.is_open())
                    PRX_FATAL_S("Could not open the file : " << file);
                
                left_IK_data_base->serialize(fout_left, _left_manipulator->get_state_space(), 4);
                
                fout_left.close();
                fout_left.clear();
                
                file = dir + "left_camera.database";
                fout_left.open(file.c_str());
                
                if(!fout_left.is_open())
                    PRX_FATAL_S("Could not open the file : " << file);
                left_cam_IK_data_base->serialize(fout_left, _left_manipulator->get_state_space(), 4);
                
                // RIGHT
                file = dir + right_IK_file;
                PRX_PRINT("Store right IK data base at the file: " << file, PRX_TEXT_CYAN);
                
                std::ofstream fout_right;
                fout_right.open(file.c_str());
                
                if(!fout_right.is_open())
                    PRX_FATAL_S("Could not open the file : " << file);
                right_IK_data_base->serialize(fout_right, _right_manipulator->get_state_space(), 4);
                
                fout_right.close();
                fout_right.clear();
                
                file = dir + "right_camera.database";
                fout_right.open(file.c_str());
                
                if(!fout_right.is_open())
                    PRX_FATAL_S("Could not open the file : " << file);
                right_cam_IK_data_base->serialize(fout_right, _right_manipulator->get_state_space(), 4);                    
            }
            
            if( !load_roadmaps )
                PRX_FATAL_S("I can now exit safely");
            
            this->root_task->link_query(retracting_task_query);
            this->root_task->link_query(lifting_task_query);
            this->root_task->link_query(task_query);
            
            // PRX_PRINT("Root Task statistics: " << this->root_task->get_statistics()->get_statistics(), PRX_TEXT_LIGHTGRAY);
            has_roadmaps = true;
            model->use_context(old_context);
        }

        bool motoman_planning_application_t::check_for_stored_trajectory( state_t* source_state, config_t ee_config )
        {
            //Let's try using the trajectory databases here
            if( _hand == "left" && left_trajectories.size() > 0 )
            {
                // PRX_PRINT("\nLooking at left-hand trajectory database.\n", PRX_TEXT_CYAN);
                bool found_traj = false;

                // PRX_PRINT("Searching for the following::: ", PRX_TEXT_CYAN);
                // PRX_PRINT("State : " << model->get_state_space()->print_point( source_state, 6 ), PRX_TEXT_LIGHTGRAY );
                // PRX_PRINT("Config: " << ee_config, PRX_TEXT_LIGHTGRAY );

                //Search for the goal we are trying to reach
                for( unsigned i=0; i<left_trajectories.size() && !found_traj; ++i )
                {
                    // PRX_PRINT("Entry: " << i, PRX_TEXT_BROWN);
                    // PRX_PRINT("State : " << model->get_state_space()->print_point( left_trajectories[i].start, 6 ), PRX_TEXT_LIGHTGRAY );
                    // PRX_PRINT("Config: " << left_trajectories[i].end_configuration, PRX_TEXT_LIGHTGRAY );

                    //If the goal is the same and we're starting from the correct state
                    if( left_trajectories[i].end_configuration.is_approximate_equal( ee_config ) &&
                        model->get_state_space()->equal_points( left_trajectories[i].start, source_state, .001) )
                    {
                        found_traj = true;
                        IK_traj = left_trajectories[i].trajectory;
                        IK_plan.clear();
                    }
                    // else
                    // {
                    //     PRX_PRINT( "Trajectory end configuration:  " << left_trajectories[i].end_configuration, PRX_TEXT_GREEN );
                    //     PRX_PRINT( "End-Effector Config:  " << ee_config, PRX_TEXT_CYAN );
                    //     PRX_PRINT( "------: "<<model->get_state_space()->distance( left_trajectories[i].start, source_state), PRX_TEXT_LIGHTGRAY );
                    // }
                }
                //If we did find a trajectory in the database, report success
                if( found_traj )
                {
                    // PRX_PRINT("\n\n\n ======= Using a trajectory from the left arm database! ===== \n\n", PRX_TEXT_GREEN);
                    return true;                    
                }
                else
                {
                    // PRX_FATAL_S("Unable to find trajectory in left arm database!");
                    // PRX_PRINT("Unable to find trajectory in left arm database!", PRX_TEXT_RED);
                    return false;
                }
            }
            else if( _hand == "right" && right_trajectories.size() > 0 )
            {
                // PRX_PRINT("\nLooking at right-hand trajectory database.\n", PRX_TEXT_CYAN);
                bool found_traj = false;
                //Search for the goal we are trying to reach
                for( unsigned i=0; i<right_trajectories.size() && !found_traj; ++i )
                {
                    //If the goal is the same and we're starting from the correct state
                    if( right_trajectories[i].end_configuration.is_approximate_equal( ee_config ) &&
                        model->get_state_space()->equal_points( right_trajectories[i].start, source_state,.001 ) )
                    {
                        found_traj = true;
                        IK_traj = right_trajectories[i].trajectory;
                        IK_plan.clear();
                    }
                    // else
                    // {
                    //     PRX_INFO_S( right_trajectories[i].end_configuration );
                    //     PRX_INFO_S( ee_config );
                    //     PRX_INFO_S( "------: "<<model->get_state_space()->distance( right_trajectories[i].start, source_state));
                    // }
                }
                //If we did find a trajectory in the database, report success
                if( found_traj )
                {
                    // PRX_PRINT("\n\n\n ======= Using a trajectory from the right arm database! ===== \n\n", PRX_TEXT_GREEN);
                    return true;
                }
                else
                {
                    // PRX_FATAL_S("Unable to find trajectory in right arm database!");
                    // PRX_PRINT("Unable to find trajectory in right arm database!", PRX_TEXT_RED);
                    return false;
                }
            }      
            return false;
        }

        void motoman_planning_application_t::load_trajectories()
        {
            // LEFT TRAJECTORIES
            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/prx_output/");

            //We need to explicily load both, so try both explicitly
            std::string file = dir + "trajectories_left.database";

            std::ifstream traj_in;
            traj_in.open( file.c_str() );

            if( !traj_in.is_open() )
            {
                PRX_FATAL_S( "Left trajectory file failed to open!" );
            }

            //We need something more principled here....
            for( unsigned i=0; i<48; ++i )
            {
                //Create a new thingermajigger in the trajectory database
                left_trajectories.push_back( trajectory_db_entry() );

                //First, deserialize the start point
                left_trajectories.back().start = model->get_state_space()->deserialize_point( traj_in );
                // PRX_PRINT("STATE: " << model->get_state_space()->print_point( left_trajectories.back().start, 4 ), PRX_TEXT_BLUE );

                //Then, we get the end configuration
                double x, y, z, w;
                char junk;
                traj_in >> x >> y >> z;
                left_trajectories.back().end_configuration.set_position( x, y, z );
                traj_in >> x >> y >> z >> w;
                left_trajectories.back().end_configuration.set_xyzw_orientation( x, y, z, w );

                // PRX_PRINT("Configuration: " << left_trajectories.back().end_configuration, PRX_TEXT_LIGHTGRAY);

                //Finally, get the trajectory
                left_trajectories.back().trajectory.link_space(_left_manipulator->get_state_space());
         
                unsigned num_traj_points;
                traj_in >> num_traj_points;
                space_point_t* state;
                // PRX_PRINT("Trajectory: " << num_traj_points, PRX_TEXT_BLUE);
                for( unsigned k=0; k<num_traj_points; ++k )
                {
                    state = model->get_state_space()->deserialize_point( traj_in );
                    // PRX_PRINT(":: " << model->get_state_space()->print_point( state, 6 ), PRX_TEXT_LIGHTGRAY );
                    left_trajectories.back().trajectory.copy_onto_back( state );
                    model->get_state_space()->free_point( state );
                }
            }
            traj_in.close();

            // PRX_PRINT("Read in " << left_trajectories.size() << " trajectories for the left arm", PRX_TEXT_CYAN);

            // RIGHT TRAJECTORIES
            //We need to explicily load both, so try both explicitly
            file = dir + "trajectories_right.database";

            traj_in.open( file.c_str() );

            if( !traj_in.is_open() )
            {
                PRX_FATAL_S( "Right trajectory file failed to open!" );
            }

            // RIGHT ARMs

            //We need something more principled here....
            for( unsigned i=0; i<48; ++i )
            {
                //Create a new thingermajigger in the trajectory database
                right_trajectories.push_back( trajectory_db_entry() );

                //First, deserialize the start point
                right_trajectories.back().start = model->get_state_space()->deserialize_point( traj_in );
                // PRX_PRINT("STATE: " << model->get_state_space()->print_point( right_trajectories.back().start, 4 ), PRX_TEXT_BLUE );

                //Then, we get the end configuration
                double x, y, z, w;
                char junk;
                traj_in >> x >> y >> z;
                right_trajectories.back().end_configuration.set_position( x, y, z );
                traj_in >> x >> y >> z >> w;
                right_trajectories.back().end_configuration.set_xyzw_orientation( x, y, z, w );

                // PRX_PRINT("Configuration: " << right_trajectories.back().end_configuration, PRX_TEXT_LIGHTGRAY);

                //Finally, get the trajectory
                right_trajectories.back().trajectory.link_space(_right_manipulator->get_state_space());
         
                unsigned num_traj_points;
                traj_in >> num_traj_points;
                space_point_t* state;
                // PRX_PRINT("Trajectory: " << num_traj_points, PRX_TEXT_BLUE);
                for( unsigned k=0; k<num_traj_points; ++k )
                {
                    state = model->get_state_space()->deserialize_point( traj_in );
                    // PRX_PRINT(":: " << model->get_state_space()->print_point( state, 6 ), PRX_TEXT_LIGHTGRAY );
                    right_trajectories.back().trajectory.copy_onto_back( state );
                    model->get_state_space()->free_point( state );
                }
            }
            traj_in.close();

            // PRX_PRINT("Read in " << right_trajectories.size() << " trajectories for the right arm", PRX_TEXT_CYAN);

        }

        void motoman_planning_application_t::init_object_geometry(std::string object_name)
        {
            geom_map_t geom_map;
            config_list_t configs;
            unsigned int count = 0;
            model->get_simulator()->update_phys_geoms(geom_map);
            model->get_simulator()->update_phys_configs(configs, count);
            unsigned found_index = 0;
            std::string system_name = "simulator/" + object_name + "/body";
            for( unsigned i = 0; i < count; i++ )
            {
                if( configs[i].first == system_name )
                {
                    found_index = i;
                    break;
                }
            }

            vis_array_pub.publish(vis_comm->send_marker(geom_map[system_name], system_name, configs[found_index].second));
        }

        void motoman_planning_application_t::check_for_context_switch(std::string the_arm)
        {
            if( _hand != the_arm )
            {
                std::string context = model->get_current_context();
                std::string::size_type i = context.find(_hand);
                if( i != std::string::npos )
                    context.erase(i, _hand.length());
                _hand = the_arm;
                model->use_context(_hand + context);
            }
            //PRX_PRINT( "I am now operating for arm : " << the_arm, PRX_TEXT_BROWN );
        }
        void operator >> (const YAML::Node& node, std::vector<double>& config) 
        {
            config.resize(7);
            config[0] = node[0].as<double>();
            config[1] = node[1].as<double>();
            config[2] = node[2].as<double>();
            config[3] = node[3].as<double>();
            config[4] = node[4].as<double>();
            config[5] = node[5].as<double>();
            config[6] = node[6].as<double>();
            
        }
        void operator >> (const YAML::Node& node, config_t& config) 
        {
            std::vector<double> vec;
            node["grasp"] >> vec;
            config.set_position(vec[0],vec[1],vec[2]);
            config.set_xyzw_orientation(vec[3],vec[4],vec[5],vec[6]);
        }

        void motoman_planning_application_t::load_grasp_data(const parameter_reader_t* reader)
        {
            PRX_PRINT("Loading Grasp Data in the Motoman Application", PRX_TEXT_MAGENTA);
            //Need to read in the UniGripper grasp data
            if( reader->has_attribute("unigripper_grasps") )
            {
                parameter_reader_t::reader_map_t uni_grasp_reader = reader->get_map("unigripper_grasps");
                
                foreach(const parameter_reader_t::reader_map_t::value_type item, uni_grasp_reader)
                {
                    std::string object_name = item.first;
                 
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string dir(w);
                    dir += ("/../grasp_data/Unigripper/")+object_name+".yaml";

                    std::ifstream fin(dir.c_str());
                    YAML::Node doc;
                    doc = YAML::Load(dir);
                    for(unsigned i=0;i<doc.size();i++) 
                    {
                        config_t config;
                        doc[i] >> config;
                        left_grasps[ object_name ].push_back( std::pair<config_t,int>(config,1) );
                    }
                    fin.close();
                    PRX_PRINT("Read [" << left_grasps[object_name].size() << "] UniGripper Grasps for object: " << object_name, PRX_TEXT_CYAN);
                }
            }
            else
            {
                PRX_WARN_S("No grasp data provided for the UniGripper: cannot grasp with the left hand!");
            }

            //Then, read in the robotiq grasp data
            if( reader->has_attribute("robotiq_grasps") )
            {
                using_robotiq_hand = true;

                parameter_reader_t::reader_map_t robo_grasp_reader = reader->get_map("robotiq_grasps");
                
                foreach(const parameter_reader_t::reader_map_t::value_type item, robo_grasp_reader)
                {
                    std::string object_name = item.first;
                 
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string dir(w);
                    dir += ("/../grasp_data/Robotiq/")+object_name+".yaml";

                    std::ifstream fin(dir.c_str());
                    YAML::Node doc;
                    doc = YAML::Load(dir);
                    for(unsigned i=0;i<doc.size();i++) 
                    {
                        config_t config;
                        doc[i] >> config;
                        right_grasps[ object_name ].push_back( std::pair<config_t,int>(config,1) );
                    }
                    fin.close();
                    PRX_PRINT("Read [" << right_grasps[object_name].size() << "] Robotiq Grasps for object: " << object_name, PRX_TEXT_LIGHTGRAY);
                }
            }

            std::vector< std::string > grasp_types;
            grasp_types.push_back("full");
            grasp_types.push_back("pinch");
            grasp_types.push_back("tips");

            //Then, read in the ReFlex grasp data
            if( reader->has_attribute("reflex_grasps") )
            {
                using_robotiq_hand = false;

                parameter_reader_t::reader_map_t ref_grasp_reader = reader->get_map("reflex_grasps");
                
                foreach(const parameter_reader_t::reader_map_t::value_type item, ref_grasp_reader)
                {
                    std::string object_name = item.first;
                 
                    //Alright, now we have to read in all of these different files
                    for( unsigned k=0; k<grasp_types.size(); ++k )
                    {
                        char* w = std::getenv("PRACSYS_PATH");
                        std::string dir(w);
                        dir += ("/../grasp_data/Reflex/")+object_name+("_")+grasp_types[k]+".yaml";

                        std::ifstream fin(dir.c_str());
                        if( fin.good() )
                        {
                            PRX_PRINT("Reading in information from file: " << dir, PRX_TEXT_LIGHTGRAY)

                            YAML::Node doc;
                            doc = YAML::Load(dir);
                            for(unsigned i=0; i<doc.size(); i++) 
                            {
                                config_t config;
                                doc[i] >> config;
                                right_grasps[ object_name ].push_back( std::pair<config_t,int>(config, k+1) );
                            }
                        }
                        fin.close();
                    }
                    PRX_PRINT("Read [" << right_grasps[object_name].size() << "] Reflex Grasps for object: " << object_name, PRX_TEXT_CYAN);
                }
            }

        }

        bool motoman_planning_application_t::find_grasp_for_object( )
        {
            // PRX_PRINT( "==================  FIND GRASP  ===== for object: " << object_name << " and arm " << _hand, PRX_TEXT_BROWN );

            bool object_in_collision = model->get_simulator()->in_collision();
            if(object_in_collision)
                return false;

            // Find the pose of the object pose
            config_t object_pose;
            sim::system_ptr_t object = object_list[object_name];
            movable_body_plant_t* cast_object = dynamic_cast< movable_body_plant_t* >( object.get() );
            if( cast_object == NULL )
                return false;
            cast_object->get_configuration( object_pose );

            //Get the relative orientations first
            std::vector< std::pair<config_t, int > >* config_pointer = NULL;            
            if( _hand == "left" )
                config_pointer = &( left_grasps[ object_name ] );
            else 
                config_pointer = &(right_grasps[ object_name ]);
            PRX_ASSERT( config_pointer != NULL );
            const std::vector< std::pair<config_t, int > >& relative_configurations = *config_pointer;

            //PRX_PRINT("Object has: [" << relative_configurations.size() << "] possible grasps.", PRX_TEXT_CYAN);
            
            config_t right_hand_config;
            config_t right_hand_pinch_config;

            if( using_robotiq_hand )
            {
                //Robotiq Configuration
                right_hand_config.set_position( 0, 0.14, 0 );
                right_hand_config.set_orientation( 0.5, -0.5, -0.5, -0.5 );
            }
            else
            {
                //ReFlex Configuration
                right_hand_config.set_position( 0.01, 0, 0.1295 );
                right_hand_config.set_orientation( 0, 0, 0, 1 );
            }

            //ReFlex Configuration
            right_hand_pinch_config.set_position( 0.01, 0, 0.14 );
            right_hand_pinch_config.set_orientation( 0, 0, 0, 1 );

            config_t left_hand_config;
            left_hand_config.set_position( 0.01, 0.0, 0.004 );
            left_hand_config.set_orientation( 0, 0, 0, 1 );           


            std::vector<unsigned> indices;
            for (int i = 0; i < relative_configurations.size(); ++i)
            {
                indices.push_back(i);
            }

            for (int i = 0; i < 9001; ++i)
            {
                unsigned i1,i2;
                i1 = uniform_int_random(0,relative_configurations.size()-1);
                i2 = uniform_int_random(0,relative_configurations.size()-1);
                std::swap(indices[i1],indices[i2]);
            }

            //Then, for each of those, put it into the global coordinate frame
            for( unsigned i=0; i<indices.size(); ++i )
            {
                if( _hand == "left" )
                {
                    grasp_configurations[i] = left_hand_config;
                    grasp_configurations[i].relative_to_global( relative_configurations[indices[i]].first );
                }
                else
                {
                    if( relative_configurations[indices[i]].second == 1 || relative_configurations[indices[i]].second == 3 )
                        grasp_configurations[i] = right_hand_config;
                    else if( relative_configurations[indices[i]].second == 2 )
                        grasp_configurations[i] = right_hand_pinch_config;
                    else 
                        PRX_FATAL_S("Improper grasp mode specified!!!  " << relative_configurations[indices[i]].second );
                    grasp_configurations[i].relative_to_global( relative_configurations[indices[i]].first );
                }

                grasp_configurations[i].relative_to_global( object_pose );
            }

            //Ask for an IK solution to that configuration
            manipulator_plant_t* manipulator;
            if( _hand == "left" )
                manipulator = _left_manipulator;
            else
                manipulator = _right_manipulator;

            model->get_active_space()->copy_to_point( object_start_state );

            steering_time = seeding_time = validation_time = planning_time = 0;
            steering_counter = validation_counter = planning_counter = 0;

            bool success = false;
            timeout_clock.reset();

            sys_clock_t zak_clock;
            zak_clock.reset();

            stop_updating_state();
            unsigned i;
            for( i=0; i<relative_configurations.size() && success == false; ++i )
            {               
                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
                // PRX_PRINT( "\n --- Trying grasp id: " << i << " ---\n", PRX_TEXT_BROWN );                
                if( timeout_clock.measure() > GRASPING_TIMEOUT )
                {
                    PRX_PRINT( "\n\n\n Grasping Timeout \n\n\n", PRX_TEXT_BROWN );
                    break;
                }

                success = reachable_IK( false, current_state, grasp_configurations[i], GRASPING_IK );
                if( success )
                    PRX_PRINT("Found grasp, id: " << indices[i] << "   type: " << relative_configurations[indices[i]].second, PRX_TEXT_CYAN);
            }
            start_updating_state(); 
            num_grasps_tried = i; 
            grasp_id = relative_configurations[indices[i-1]].second;          

            // PRX_PRINT("overall_time " << zak_clock.measure(), PRX_TEXT_BROWN );

            //PRX_PRINT(" steering_time " << steering_time, PRX_TEXT_BROWN );
            //PRX_PRINT(" seeding_time " << seeding_time, PRX_TEXT_BROWN );
            //PRX_PRINT(" validation_time " << validation_time, PRX_TEXT_BROWN );
            //PRX_PRINT(" planning_time " << planning_time, PRX_TEXT_BROWN );

            //PRX_PRINT(" avarage steering_time " << (steering_time/steering_counter), PRX_TEXT_BROWN );
            //PRX_PRINT(" avarage seeding_time " << (seeding_time/validation_counter), PRX_TEXT_BROWN );
            //PRX_PRINT(" avarage validation_time " << (validation_time/validation_counter), PRX_TEXT_BROWN );
            //PRX_PRINT(" avarage planning_time " << (planning_time/planning_counter), PRX_TEXT_BROWN );

            return success;
        }

        bool motoman_planning_application_t::compute_grasp_callback(prx_planning::compute_grasp::Request& req, prx_planning::compute_grasp::Response& res )
        {
            object_name = req.object_name;
            if( find_grasp_for_object( ) )
            {
                res.success = true;
                res.arm = _hand;
                res.grasp_id = grasp_id;
                res.num_grasps_tried = num_grasps_tried;
                return true;
            }
            else 
            {
                PRX_PRINT( "Grasping failed - REPORTING FAILURE", PRX_TEXT_BROWN );
                res.success = false;
                res.arm = _hand;
                res.num_grasps_tried = num_grasps_tried;
                return true;
            }
        }

        void motoman_planning_application_t::impose_hand_state( space_point_t* target_state )
        {
            if( _hand == "left" )
            {
                for( int i=8; i<15; i++ )
                    target_state->at(i) = current_state->at(i);
            }
            else
            {
                for( int i=1; i<8; i++ )
                    target_state->at(i) = current_state->at(i);
                target_state->at(15) = current_state->at(15);
            }
            target_state->at(16) = _grasping;
        }
        

        // camera_link is the
        // source_state is the current state of the robot
        // ee_config is the desired end effector configuration
        // target_state is the state corresponding to the end effector
        // resulting_plan is the trajectory from source_state to the target_state
        // IK_stage: one of
        bool motoman_planning_application_t::reachable_IK( bool camera_link, space_point_t* source_state, const config_t& ee_config, int IK_stage )
        {
            // PRX_PRINT( "reachable IK function at IK stage:: " << IK_stage, PRX_TEXT_BROWN + IK_stage );
            //PRX_PRINT( "source state = " << model->get_state_space()->print_point( source_state, 3 ), PRX_TEXT_BROWN + IK_stage );

            if( timeout_clock.measure() > GRASPING_TIMEOUT )
            {
                PRX_PRINT( "\n\n\n Grasping Timeout \n\n\n", PRX_TEXT_BROWN );
                return false;
            }

            if( camera_link == true && IK_stage != GENERAL_IK )
            {
                PRX_PRINT( "\nA grasping/lifting/retraction IK call for a camera - ERROR\n", PRX_TEXT_RED);
                exit(-1);
            }
            if( IK_stage != GENERAL_IK && IK_stage != GRASPING_IK && IK_stage != LIFTING_IK && IK_stage != RETRACTION_IK )
            {
                PRX_PRINT( "\nInvalid IK stage parameter: " << IK_stage << "  - ERROR\n", PRX_TEXT_RED);
                exit(-1);
            }

            if (IK_stage == GRASPING_IK)
            {
                if(_hand == "left")
                {
                    _left_manipulator->get_state_space()->copy_from_point(source_state);
                    _left_manipulator->get_end_effector_configuration(retracting_config);
                }
                else
                {
                    _right_manipulator->get_state_space()->copy_from_point(source_state);
                    _right_manipulator->get_end_effector_configuration(retracting_config);
                }
            }

            //IK_clock.reset();

                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
            // First, I will try IK steering
            bool success = check_IK_steering( camera_link, source_state, ee_config, IK_stage );
            
                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
            //steering_counter++;
            //double local_steering_time = IK_clock.measure_reset();
            //PRX_PRINT( "steering time = " << local_steering_time, PRX_TEXT_BROWN + IK_stage );            
            //steering_time += local_steering_time;

            if( !success && IK_stage != LIFTING_IK )
            {
                generate_IK_seeds( camera_link );

                //validation_counter++;
                //double local_seeding_time = IK_clock.measure_reset();
                //PRX_PRINT( "seeding time = " << local_seeding_time, PRX_TEXT_BROWN + IK_stage );
                //seeding_time += local_seeding_time;

                int number_of_valid_states = generate_valid_IK_states( camera_link, ee_config, IK_stage );

                //double local_validation_time = IK_clock.measure_reset();
                //PRX_PRINT( "validation time = " << local_validation_time, PRX_TEXT_BROWN + IK_stage );
                //validation_time += local_validation_time;

                if( number_of_valid_states > 0 )
                {
                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
                    //planning_counter++;
                    success = compute_reachable_IK( source_state, ee_config, IK_stage, number_of_valid_states );
                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
                }
                else
                    success = false;

                //double local_planning_time = IK_clock.measure_reset();
                //PRX_PRINT( "planning time = " << local_planning_time, PRX_TEXT_BROWN + IK_stage );
                //planning_time += local_planning_time;
            }

            return success;
        }

        void motoman_planning_application_t::generate_IK_seeds( bool for_camera )
        {
            //Check that the memory is allocated
            for( int i=0; i<IK_seeds.size(); i++ )
                if( IK_seeds[i] == NULL )
                    IK_seeds[i] = model->get_state_space()->alloc_point();

            IK_data_base_t* IK_DB;
            if( for_camera && _hand == "left" )
                IK_DB = left_cam_IK_data_base;
            else if( !for_camera && _hand == "left" )
                IK_DB = left_IK_data_base;
            else if( for_camera && _hand == "right" )
                IK_DB = right_cam_IK_data_base;
            else if( !for_camera && _hand == "right" )
                IK_DB = right_IK_data_base;

            // RANDOM Seeds Version
            if( IK_DB == NULL )
            {
                PRX_FATAL_S("Random IK Seeds" );
                model->get_state_space()->copy_point( IK_seeds[0], current_state );
                impose_hand_state( IK_seeds[0] );

                for( int i=1; i<IK_seeds.size(); i++ )
                {
                    model->get_state_space()->uniform_sample( IK_seeds[i] );
                    impose_hand_state( IK_seeds[i] );
                }
            }
            //Database Seeds Version
            else
            {
                config_t ee_config;
                manipulator_plant_t* manipulator = (_hand == "left") ? _left_manipulator : _right_manipulator;
                manipulator->get_end_effector_configuration( ee_config );
                
                std::vector< state_t* > local_seeds;
                IK_DB->get_near_neighbors( local_seeds, ee_config, IK_seeds.size()-1 );
                
                model->get_state_space()->copy_point( IK_seeds[0], current_state );
                for( unsigned i=0; i<local_seeds.size(); ++i )
                    model->get_state_space()->copy_point( IK_seeds[i+1], local_seeds[i] );
            }
        }

        int motoman_planning_application_t::generate_valid_IK_states( bool camera_link, const config_t& config, int IK_stage )
        {
            manipulator_plant_t* manipulator = ( _hand == "left" ? _left_manipulator : _right_manipulator );

            if( timeout_clock.measure() > GRASPING_TIMEOUT )
            {
                PRX_PRINT( "\n\n\n Grasping Timeout \n\n\n", PRX_TEXT_BROWN );
                return 0;
            }

            std::vector< state_t* >* local_IK_ptr;
            if( IK_stage == GENERAL_IK || IK_stage == GRASPING_IK )
                local_IK_ptr = &candidate_IKs;
            else if( IK_stage == LIFTING_IK )
                local_IK_ptr = &lifting_IKs;
            else if ( IK_stage == RETRACTION_IK )
                local_IK_ptr = &retraction_IKs;
            std::vector< state_t* > &local_candidate_IKs = *local_IK_ptr;

            unsigned number_of_valid_states = 0;
            int in_collision = 0;
            int failed_ik = 0;
            for( unsigned i=0; i<IK_seeds.size(); ++i )
            {
                if( timeout_clock.measure() > GRASPING_TIMEOUT )
                {
                    PRX_PRINT( "\n\n\n Grasping Timeout \n\n\n", PRX_TEXT_BROWN );
                    break;
                }

                bool success = false;
                //PRX_PRINT( "Calling for link " << camera_link, PRX_TEXT_BROWN + IK_stage );
                if( camera_link )
                    success = manipulator->get_utility_IK( config, local_candidate_IKs[number_of_valid_states], _grasping, IK_seeds[i], true );
                else
                    success = manipulator->IK_solver( config, local_candidate_IKs[number_of_valid_states], _grasping, IK_seeds[i], true );

                if( visualize_grasps && success && IK_stage == GRASPING_IK )
                {
                    std::vector<double> params;
                    params.push_back(.02);
                    geometry_t geom(PRX_SPHERE,&params);
                    
                    vis_array_pub.publish(vis_comm->send_marker(geom, "THING_config", config));                    
                }

                //PRX_PRINT( "IK success is " << success << "for state: " << model->get_state_space()->print_point( local_candidate_IKs[number_of_valid_states], 4 ), PRX_TEXT_BROWN + IK_stage );
                if( success )
                {
                    if( model->valid_state( local_candidate_IKs[number_of_valid_states] ) )
                        ++number_of_valid_states;
                    else
                    {
                        in_collision++;
                        // PRX_PRINT( "The resulting state is in collision", PRX_TEXT_BROWN + IK_stage );

                        // collision_list_t* list = model->get_colliding_bodies();
                        // foreach(collision_pair_t pair, list->get_body_pairs())
                        // {
                        //    PRX_WARN_S(pair.first << " " << pair.second);
                        // }
                        // PRX_WARN_S("");
                    }
                }
                else
                    failed_ik++;

                if ( IK_stage == RETRACTION_IK && number_of_valid_states >= 25 )
                    break;
            }

            //If there are no valid free states, abort out, no need to do more work
            // if( number_of_valid_states == 0 )
            // {
            //     PRX_PRINT( "SEEDS FAILED: out of (" << IK_seeds.size() << "):  IK failures: " << failed_ik << "  Collisions: " << in_collision , PRX_TEXT_BROWN + IK_stage );
            // }
            // else
            // {
            //     PRX_PRINT( "NUMBER OF VALID STATES IS " << number_of_valid_states, PRX_TEXT_BROWN + IK_stage );
            // }

            return number_of_valid_states;
        }

        bool motoman_planning_application_t::compute_reachable_IK( space_point_t* source_state, const config_t& ee_config, int IK_stage, int number_of_valid_states )
        {
            if( timeout_clock.measure() > GRASPING_TIMEOUT )
            {
                PRX_PRINT( "\n\n\n Grasping Timeout \n\n\n", PRX_TEXT_BROWN );
                return false;
            }
/*
            std::vector< trajectory_t > successful_trajectories;
            std::vector< plan_t > successful_plans;
            std::vector< space_point_t* > successful_states; 
*/

            std::vector< state_t* > local_candidate_IKs;
            for( int i=0; i<number_of_valid_states; i++ )
            {
                if( IK_stage == GENERAL_IK || IK_stage == GRASPING_IK )
                    local_candidate_IKs.push_back( candidate_IKs[i] );
                else if( IK_stage == LIFTING_IK )
                    local_candidate_IKs.push_back( lifting_IKs[i] );
                else if ( IK_stage == RETRACTION_IK )
                    local_candidate_IKs.push_back( retraction_IKs[i] );
            }

            //PRX_PRINT( "the size of the vector is " << local_candidate_IKs.size(), PRX_TEXT_BROWN + IK_stage );

                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
            if( find_shortest_path_internal( source_state, local_candidate_IKs, IK_stage ) )
            {
                if( IK_stage == GENERAL_IK )
                {
                    IK_plan = task_query->plan;     
                    IK_traj = task_query->path;   
                    
                    // PRX_PRINT( "Computed plan for :: ",  PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Initial state :: " <<  model->get_state_space()->print_point( source_state, 3 ) , PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Goal state :: " <<  model->get_state_space()->print_point( task_query->path.back(), 3 ), PRX_TEXT_BROWN + IK_stage );

                    return true;
                }
                else if( IK_stage == GRASPING_IK )
                {
                    int counter = local_candidate_IKs.size();
                    while( counter > 0 )
                    {
                        if( timeout_clock.measure() > GRASPING_TIMEOUT )
                        {
                            PRX_PRINT( "\n\n\n Grasping Timeout \n\n\n", PRX_TEXT_BROWN );
                            return false;
                        }

                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
                        if( IK_recursion( task_query->path.back(), ee_config, GRASPING_IK ) )
                        {
                            grasping_traj = task_query->path;
                            grasping_plan = task_query->plan;

                            // PRX_PRINT( "Computed plan for :: ",  PRX_TEXT_BROWN + IK_stage );
                            //PRX_PRINT( "Initial state :: " <<  model->get_state_space()->print_point( source_state, 3 ) , PRX_TEXT_BROWN + IK_stage );
                            //PRX_PRINT( "Goal state :: " <<  model->get_state_space()->print_point( task_query->path.back(), 3 ), PRX_TEXT_BROWN + IK_stage );

                            return true;
                        }
                        else
                        {
                            int goal_index = -1;
                            for( int i=0; i<local_candidate_IKs.size() && goal_index == -1; i++ )
                                if( model->get_state_space()->equal_points( local_candidate_IKs[i], task_query->path.back(), .001) )
                                    goal_index = i;
                            if( goal_index == -1 )
                                PRX_FATAL_S( "The last state on the trajectory was not in the local candidate IK vector" );
                            local_candidate_IKs.erase( local_candidate_IKs.begin() + goal_index );
                            counter--;
                            if( find_shortest_path_internal( source_state, local_candidate_IKs, GRASPING_IK ) == false )
                                return false;
                        }
                    }
                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
                    return false;
                }
                else if( IK_stage == LIFTING_IK )
                {
                    int counter = local_candidate_IKs.size();
                    while( counter > 0 )
                    {
                        if( timeout_clock.measure() > GRASPING_TIMEOUT )
                        {
                            PRX_PRINT( "\n\n\n Grasping Timeout \n\n\n", PRX_TEXT_BROWN );
                            return false;
                        }

                        if( IK_recursion( lifting_task_query->path.back(), ee_config, LIFTING_IK ) )
                        {
                            lifting_traj = lifting_task_query->path;
                            lifting_plan = lifting_task_query->plan;

                    // PRX_PRINT( "Computed plan for :: ",  PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Initial state :: " <<  model->get_state_space()->print_point( source_state, 3 ) , PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Goal state :: " <<  model->get_state_space()->print_point( lifting_task_query->path.back(), 3 ), PRX_TEXT_BROWN + IK_stage );

                            return true;
                        }
                        else
                        {
                            int goal_index = -1;
                            for( int i=0; i<local_candidate_IKs.size() && goal_index == -1; i++ )
                                if( model->get_state_space()->equal_points( local_candidate_IKs[i], lifting_task_query->path.back(), .001) )
                                    goal_index = i;
                            if( goal_index == -1 )
                                PRX_FATAL_S( "The last state on the trajectory was not in the local candidate IK vector" );
                            local_candidate_IKs.erase( local_candidate_IKs.begin() + goal_index );
                            counter--;
                            if( find_shortest_path_internal( source_state, local_candidate_IKs, LIFTING_IK ) == false )
                                return false;
                        }
                    }
                    return false;
                }
                else if( IK_stage == RETRACTION_IK )
                {
                    retracting_plan = retracting_task_query->plan;
                    retracting_traj = retracting_task_query->path;  

                    // PRX_PRINT( "Computed plan for :: " << retracting_traj.size(),  PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Initial state :: " <<  model->get_state_space()->print_point( source_state, 3 ) , PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Goal state :: " <<  model->get_state_space()->print_point( retracting_task_query->path.back(), 3 ), PRX_TEXT_BROWN + IK_stage );

                    return true;
                }
            }
            else
                return false;

/*
            std::vector< state_t* >* local_IK_ptr;
            if( IK_stage == GENERAL_IK || IK_stage == GRASPING_IK )
                local_IK_ptr = &candidate_IKs;
            else if( IK_stage == LIFTING_IK )
                local_IK_ptr = &lifting_IKs;
            else if ( IK_stage == RETRACTION_IK )
                local_IK_ptr = &retraction_IKs;
            std::vector< state_t* > &local_candidate_IKs = *local_IK_ptr;

            bool success = false;
            for( unsigned i=0; i<number_of_valid_states && success == false; i++ )
            {
                if( timeout_clock.measure() > GRASPING_TIMEOUT )
                {
                    PRX_PRINT( "\n\n\n Grasping Timeout \n\n\n", PRX_TEXT_BROWN );
                    return false;
                }

                // PRX_PRINT(" -- Checking valid state " << i << " in mode " << IK_stage, PRX_TEXT_BROWN + IK_stage );

// For Multi-goal use:: find_shortest_path_internal( source_state, candidate_IKs );
                if( IK_stage == GENERAL_IK && find_path_internal( source_state, local_candidate_IKs[i], GENERAL_IK ) )
                {
                    successful_trajectories.push_back( task_query->path );
                    successful_plans.push_back( task_query->plan );
                    successful_states.push_back( local_candidate_IKs[i] );
                }
                else if( IK_stage == GRASPING_IK && find_path_internal( source_state, local_candidate_IKs[i], GRASPING_IK ) )
                {
                    if( IK_recursion( local_candidate_IKs[i], ee_config, GRASPING_IK ) )
                    {
                        successful_trajectories.push_back( task_query->path );
                        successful_plans.push_back( task_query->plan );
                        successful_states.push_back( local_candidate_IKs[i] );
                        success = true;
                    }
                }
                else if( IK_stage == LIFTING_IK && find_path_internal( source_state, local_candidate_IKs[i], LIFTING_IK  ) )
                {
                    if( IK_recursion( local_candidate_IKs[i], ee_config, LIFTING_IK) )
                    {
                        successful_trajectories.push_back( lifting_task_query->path );
                        successful_plans.push_back( lifting_task_query->plan );
                        successful_states.push_back( local_candidate_IKs[i] );
                        success = true;
                    }
                }
                else if( IK_stage == RETRACTION_IK && find_path_internal( source_state, local_candidate_IKs[i], RETRACTION_IK ))
                {
                    successful_trajectories.push_back( retracting_task_query->path );
                    successful_plans.push_back( retracting_task_query->plan );
                    successful_states.push_back( local_candidate_IKs[i] );
                    success = true;
                    // PRX_PRINT( "WE HAVE LIFT OFF!!!!!", PRX_TEXT_GREEN );
                }
            }
            PRX_PRINT( "Number of successful states is " << successful_trajectories.size(), PRX_TEXT_BROWN + IK_stage );

            double smallest_cost = PRX_INFINITY;
            unsigned shortest_index;
            if( successful_trajectories.size() > 0 )
            {
                //For each one, evaluate it based on its length, remembering the best one
                for( unsigned i=0; i<successful_trajectories.size(); ++i )
                {
                    if( successful_trajectories[i].length() < smallest_cost )
                    {
                        shortest_index = i;
                        smallest_cost = successful_trajectories[i].length();
                    }
                }
                
                //PRX_PRINT( "Successful state :: " << model->get_state_space()->print_point( successful_states[shortest_index], 3 ), PRX_TEXT_BROWN + IK_stage );
                //PRX_PRINT( "With index :: " << shortest_index, PRX_TEXT_BROWN + IK_stage );

                //PRX_INFO_S("Selected trajectory: \n" << successful_trajectories[shortest_index].print());
                
                // these variables can be used when we plan
                if( IK_stage == GENERAL_IK )
                {
                    IK_plan = successful_plans[ shortest_index ];     
                    IK_traj = successful_trajectories[shortest_index];           
                }
                else if(IK_stage == GRASPING_IK)
                {
                    grasping_plan = successful_plans[ shortest_index ];
                    grasping_traj = successful_trajectories[shortest_index];   
                }
                else if(IK_stage == LIFTING_IK)
                {
                    lifting_plan = successful_plans[ shortest_index ];
                    lifting_traj = successful_trajectories[shortest_index];   
                }
                else if(IK_stage == RETRACTION_IK)
                {
                    retracting_plan = successful_plans[ shortest_index ];
                    retracting_traj = successful_trajectories[shortest_index];   
                }
                return true;                                
            }
            
            return false;
*/
        }

        bool motoman_planning_application_t::IK_recursion( space_point_t* destination, const config_t& destination_config, int IK_stage )
        {
            if( timeout_clock.measure() > GRASPING_TIMEOUT )
            {
                PRX_PRINT( "\n\n\n Grasping Timeout \n\n\n", PRX_TEXT_BROWN );
                return false;
            }

            if( IK_stage == GRASPING_IK )
            {
//                PRX_PRINT("In Grasping mode - making a recursive call", PRX_TEXT_BROWN + IK_stage );
                
                config_t lifting_config;
                lifting_config = destination_config;
                double x,y,z;
                lifting_config.get_position( &x, &y, &z );
                lifting_config.set_position_at( 2, z+LIFT_HEIGHT ); 
                
                //PRX_PRINT("Lifting configuration is: " << lifting_config, PRX_TEXT_BROWN + IK_stage );
                
                model->use_context(_hand + "_" + object_name);
                _grasping = true;
                
                destination->at(16) = 1;
                model->push_state( destination );
                
                model->get_active_space()->copy_from_point( object_start_state );
                
                //PRX_PRINT(" THE OBJECT IS AT 1 :::: " << model->get_active_space()->print_memory( 7 ), PRX_TEXT_BROWN + IK_stage);
                ((manipulation_simulator_t*)model->get_simulator())->compute_relative_config();
                
                //PRX_PRINT( "RELATIVE CONFIG 1", PRX_TEXT_BROWN + IK_stage );
                //((manipulation_simulator_t*)model->get_simulator())->print_relative_config();

                //PRX_PRINT( "START state is" << model->get_state_space()->print_point( destination, 3 ), PRX_TEXT_BROWN + IK_stage );
                
                bool success = reachable_IK( false, destination, lifting_config, LIFTING_IK );
                
                //PRX_PRINT( "DESTINATION 2 is" << model->get_state_space()->print_point( destination ), PRX_TEXT_BROWN + IK_stage );
                
                //PRX_PRINT( "RELATIVE CONFIG 2", PRX_TEXT_BROWN + IK_stage );
                //((manipulation_simulator_t*)model->get_simulator())->print_relative_config();
                
                model->push_state( destination );
                //PRX_PRINT(" THE OBJECT IS AT 2:::: " << model->get_active_space()->print_memory( 7 ), PRX_TEXT_BROWN + IK_stage );
                destination->at(16) = 0;
                
                model->use_context(_hand + "_" + "manipulator");
                _grasping = false;
                
                model->push_state( destination );
                ((manipulation_simulator_t*)model->get_simulator())->compute_relative_config();

                if( success && _hand == "left" )
                {
                    // PRX_PRINT("\n\n TRYING TO PUSH\n\n", PRX_TEXT_BROWN);

                    config_t initial_config;
                    _left_manipulator->get_end_effector_configuration( initial_config );

                    config_t pushing_config;
                    pushing_config.set_position( 0.0, 0.0, 0.02 );
                    pushing_config.set_orientation( 0.0, 0.0, 0.0, 1.0 );

                    pushing_config.relative_to_global( initial_config );

                    // PRX_PRINT( "initial configuration: " << initial_config, PRX_TEXT_GREEN );
                    // PRX_PRINT( "goal configuration: " << pushing_config, PRX_TEXT_GREEN );

                    plan_t first_plan, second_plan;
                    trajectory_t first_traj, second_traj;
                    first_plan.link_control_space( _left_manipulator->get_control_space() );
                    first_plan.clear();
                    first_traj.link_space( _left_manipulator->get_state_space() );
                    first_traj.clear();
                    second_plan.link_control_space( _left_manipulator->get_control_space() );
                    second_plan.clear();
                    second_traj.link_space( _left_manipulator->get_state_space() );
                    second_traj.clear();

                    if( _left_manipulator->IK_steering_general( initial_config, pushing_config, first_plan, false, false ) )
                    {
                        // PRX_PRINT("\n\n Steering General worked the first time\n\n", PRX_TEXT_BROWN);
                        for(int i=0;i<first_plan.size();i++)
                        {
                            first_plan[i].duration *= 2;
                        }

                        model->propagate_plan( destination, first_plan, first_traj, false );
                        if( _left_manipulator->IK_steering_general( pushing_config, initial_config, second_plan, false, false ) )
                        {
                            model->propagate_plan( first_traj.back(), second_plan, second_traj, false );


                            // PRX_PRINT("\n\n Steering General worked the second time\n\n", PRX_TEXT_BROWN);

                            first_plan += second_plan;
                            first_plan += lifting_plan;
                            lifting_plan = first_plan;

                            first_traj += second_traj;
                            first_traj += lifting_traj;
                            lifting_traj = first_traj;
                        }
                    }
                }
                model->use_context(_hand + "_" + object_name);
                
                return success;
            }
            else if( IK_stage == LIFTING_IK )
            {
                //PRX_PRINT("In Lifting mode - making a recursive call", PRX_TEXT_BROWN + IK_stage );
                
                config_t local_retraction_config;

                if( uniform_random( 0.0, 1.0 ) < 0.5 )
                {
                    // PRX_PRINT( "CASE gripper position and orientation", PRX_TEXT_BROWN + IK_stage );
                    double x,y,z;
                    retracting_config.get_position(x,y,z);
                    local_retraction_config.set_position( x,y,z ); 
                    double w;
                    quaternion_t quat = retracting_config.get_orientation();
                    quat.get( x, y, z, w );
                    local_retraction_config.set_orientation( x,y,z,w ); 
                }
                else
                {
                    // PRX_PRINT( "CASE lifting orientation with gripper x", PRX_TEXT_BROWN + IK_stage );

                    local_retraction_config = destination_config;
                    double x,y,z, x_new, y_new, z_new;
                    destination_config.get_position(x,y,z);
                    retracting_config.get_position(x_new,y_new,z_new);
                    local_retraction_config.set_position( x_new, y, z ); 
                }
                    
                //PRX_PRINT("Retraction configuration is: " << local_retraction_config, PRX_TEXT_BROWN + IK_stage );

                //PRX_PRINT( "START state is" << model->get_state_space()->print_point( destination, 3 ), PRX_TEXT_BROWN + IK_stage );
                
                return reachable_IK( false, destination, local_retraction_config, RETRACTION_IK );
            }
        }

        bool motoman_planning_application_t::check_IK_steering( bool camera_link, space_point_t* source_state, const config_t& ee_config, int IK_stage )
        {
            if( timeout_clock.measure() > GRASPING_TIMEOUT )
            {
                PRX_PRINT( "\n\n\n Grasping Timeout \n\n\n", PRX_TEXT_BROWN );
                return false;
            }

            //PRX_PRINT( "checking IK steering", PRX_TEXT_BROWN + IK_stage  );
            //PRX_PRINT( "Source state in CIKS: " << model->get_state_space()->print_point( source_state, 3 ),  PRX_TEXT_BROWN + IK_stage );
            //PRX_PRINT( "Destination config in CIKS: " << ee_config, PRX_TEXT_BROWN + IK_stage );

            // setup the right query, its start state and the corresponding planner
            base_apc_task_query_t* current_query = setup_planning( source_state, IK_stage );

            manipulator_plant_t* manipulator = NULL;
            if( _hand == "left" )
                manipulator = _left_manipulator;
            else
                manipulator = _right_manipulator;
            manipulator->get_state_space()->copy_from_point( source_state );
            config_t start_config;
            manipulator->get_end_effector_configuration( start_config );

            if( ee_config.is_approximate_equal( start_config ) )
            {
                current_query->plan.clear();
                current_query->path.clear();

                //PRX_PRINT( "Start and goal configurations are the same - success!!", PRX_TEXT_BROWN + IK_stage );
                return true;
            }

            // linking the query
            root_task->link_query( current_query );

            //PRX_PRINT( "Linked query and ready to perform IK resolution...", PRX_TEXT_BROWN + IK_stage );
            
            // calling the IK steering function for the desired end effector configuration
            ((base_apc_task_planner_t*)root_task)->IK_resolve_query( ee_config, _grasping, camera_link );

            // Fail to compute a path
            if( (current_query->plan.length() <= 0 ) )
            {
                // PRX_PRINT("IK steering: There was NO path", PRX_TEXT_BROWN + IK_stage );
                return false;
            }
            else
            {
                // PRX_PRINT( "IK steering: Success", PRX_TEXT_BROWN + IK_stage );
                if( IK_stage == GENERAL_IK )
                { 
                    IK_plan = task_query->plan;     
                    IK_traj = task_query->path;   
        
                    // PRX_PRINT( "Computed IK steering plan for mode ",  PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Initial state :: " <<  model->get_state_space()->print_point( source_state, 3 ) , PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Goal state :: " <<  model->get_state_space()->print_point( task_query->path.back(), 3 ), PRX_TEXT_BROWN + IK_stage );  
                    return true;
                }
                else if(IK_stage == GRASPING_IK)
                {
                    if( IK_recursion( current_query->get_goal()->get_goal_points()[0], ee_config, IK_stage ) )
                    {
                        grasping_plan = task_query->plan;     
                        grasping_traj = task_query->path;   

                        // PRX_PRINT( "Computed IK steering plan for :: ",  PRX_TEXT_BROWN + IK_stage );
                        //PRX_PRINT( "Initial state :: " <<  model->get_state_space()->print_point( source_state, 3 ) , PRX_TEXT_BROWN + IK_stage );
                        //PRX_PRINT( "Goal state :: " <<  model->get_state_space()->print_point( task_query->path.back(), 3 ), PRX_TEXT_BROWN + IK_stage );  
                        return true;
                    }
                    else
                        return false;
                }
                else if(IK_stage == LIFTING_IK)
                {
                    if( IK_recursion( current_query->get_goal()->get_goal_points()[0], ee_config, IK_stage ) )
                    {
                        lifting_plan = lifting_task_query->plan;     
                        lifting_traj = lifting_task_query->path;    

                    // PRX_PRINT( "Computed IK steering plan for :: ",  PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Initial state :: " <<  model->get_state_space()->print_point( source_state, 3 ) , PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Goal state :: " <<  model->get_state_space()->print_point( lifting_task_query->path.back(), 3 ), PRX_TEXT_BROWN + IK_stage );
                        return true;
                    }
                    else
                        return false;
                }
                else if(IK_stage == RETRACTION_IK)
                {
                    retracting_plan = retracting_task_query->plan;
                    retracting_traj = retracting_task_query->path;

                    // PRX_PRINT( "Computed IK steering plan for :: ",  PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Initial state :: " <<  model->get_state_space()->print_point( source_state, 3 ) , PRX_TEXT_BROWN + IK_stage );
                    //PRX_PRINT( "Goal state :: " <<  model->get_state_space()->print_point( retracting_task_query->path.back(), 3 ), PRX_TEXT_BROWN + IK_stage );

                    return true;
                }
            }
        }

        base_apc_task_query_t* motoman_planning_application_t::setup_planning( space_point_t* source, int IK_stage )
        {
            // setting object to its pose relative to the manipulator
            if( _grasping )
                model->push_state( source );

            base_apc_task_query_t* current_query;
            //setup new query from request
            if( IK_stage == GENERAL_IK || IK_stage == GRASPING_IK )
                current_query = task_query;
            else if( IK_stage == LIFTING_IK )
                current_query = lifting_task_query;
            else if( IK_stage == RETRACTION_IK )
                current_query = retracting_task_query;                
            current_query->clear();

            ((base_apc_task_query_t*)current_query)->clear_goal();

            // Setting the start and the goal state of the query
            model->get_state_space()->copy_point( current_query->get_start_state(), source);
            current_query->get_start_state()->at(16) = (double)_grasping;

            // Setting the planner to the appropriate one per hand use and grasping mode
            if( _hand == "left" && _grasping )
                ( (base_apc_task_planner_t*)root_task )->change_queried_planner(left_grasped_planner);
            if( _hand == "left" && !_grasping )
                ( (base_apc_task_planner_t*)root_task )->change_queried_planner(left_ungrasped_planner);
            if( _hand == "right" && _grasping )
                ( (base_apc_task_planner_t*)root_task )->change_queried_planner(right_grasped_planner);
            if( _hand == "right" && !_grasping )
                ( (base_apc_task_planner_t*)root_task )->change_queried_planner(right_ungrasped_planner);
           
            return current_query;
        }

        
        bool motoman_planning_application_t::find_path_internal( space_point_t* source, space_point_t* destination, int IK_stage )
        {
            //PRX_PRINT( "find path internal", PRX_TEXT_BROWN + IK_stage  );

            //PRX_PRINT( "Source state in FPI: " << model->get_state_space()->print_point( source, 3 ),  PRX_TEXT_BROWN + IK_stage );
            //PRX_PRINT( "Destination state in FPI: " << model->get_state_space()->print_point( destination, 3), PRX_TEXT_BROWN + IK_stage );

            base_apc_task_query_t* current_query = setup_planning( source, IK_stage );

            ( (multiple_goal_states_t*) (current_query->get_goal() ) )->init_with_goal_state( destination );
            state_t* prx_goal_state = current_query->get_goal()->get_goal_points()[0];
            prx_goal_state->at(16) = (double)_grasping;

            if( model->get_state_space()->equal_points( current_query->get_start_state(), prx_goal_state, .001) )
            {
                //PRX_PRINT( "Already at goal.  No path to be found.", PRX_TEXT_BROWN + IK_stage );
                current_query->plan.clear();
                current_query->path.clear();
                return true;
            }

            // linking the query  - to update the goal
            root_task->link_query(current_query);

            //PRX_PRINT("Resolving query in context " << model->get_current_context() << " ...",  PRX_TEXT_BROWN + IK_stage  );

            //if( _grasping )
            //    PRX_PRINT( "Before resolve query, the object is at: " << model->get_active_space()->print_memory( 7 ),  PRX_TEXT_BROWN + IK_stage  );
            //PRX_PRINT( "RELATIVE CONFIG",  PRX_TEXT_BROWN + IK_stage  );
            //((manipulation_simulator_t*)model->get_simulator())->print_relative_config();

            ((base_apc_task_planner_t*)root_task)->resolve_query( _grasping );

            //PRX_PRINT( "Path computed::\n " << current_query->path.print(),   PRX_TEXT_BROWN + IK_stage  );

            //PRX_PRINT( "RELATIVE CONFIG", PRX_TEXT_GREEN );
            //((manipulation_simulator_t*)model->get_simulator())->print_relative_config();

            //if( _grasping )
            //    PRX_PRINT( "After resolve query, the object is at: " << model->get_active_space()->print_memory( 7 ),  PRX_TEXT_BROWN + IK_stage );

            if( current_query->plan.length() <= 0 )
            {
//                PRX_PRINT("Resolve Query: There was NO path", PRX_TEXT_BROWN + IK_stage );
                return false;
            }
            else
            {
//                PRX_PRINT("Resolve Query: Path returned", PRX_TEXT_BROWN + IK_stage );
                return true;
            }
        }

        bool motoman_planning_application_t::find_shortest_path_internal( space_point_t* source, std::vector<space_point_t*> destinations, int IK_stage )
        {
            //PRX_PRINT( "find shortest path internal", PRX_TEXT_BROWN + IK_stage  );
            //PRX_PRINT( "Source state in FPI: " << model->get_state_space()->print_point( source, 3 ),  PRX_TEXT_BROWN + IK_stage );
            //PRX_PRINT( "Destination state in FPI: " << model->get_state_space()->print_point( destination, 3), PRX_TEXT_BROWN + IK_stage );

                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
            base_apc_task_query_t* current_query = setup_planning( source, IK_stage );

                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
            //PRX_PRINT( "Ready to add destinations",  PRX_TEXT_BROWN + IK_stage );
 
            foreach(state_t* d, destinations)
            {
                model->get_state_space()->copy_point( multi_goal_assist, d);
                multi_goal_assist->at(16) = (double)_grasping;
                static_cast<multiple_goal_states_t*>(current_query->get_goal())->add_goal_state(multi_goal_assist);

                //PRX_PRINT( "Goal state in FPI " << model->get_state_space()->print_point( multi_goal_assist, 3 ),  PRX_TEXT_BROWN + IK_stage );
                
                if( model->get_state_space()->equal_points( current_query->get_start_state(), multi_goal_assist, .001) )
                {
                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
                        current_query->plan.clear();
                        current_query->path.clear();
                        return true;
                }
            }
                       
            // linking the query
            root_task->link_query(current_query);

            //PRX_PRINT("Resolving query in context " << model->get_current_context() << " ...", PRX_TEXT_LIGHTGRAY );

                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
            ((base_apc_task_planner_t*)root_task)->resolve_query( _grasping );

                // PRX_INFO_S("ZL: "<<model->get_active_space()->print_memory(3));
            if( current_query->plan.length() <= 0 )
            {
                //PRX_PRINT("Resolve Query: There was NO path", PRX_TEXT_BROWN + IK_stage );
                return false;
            }
            else
            {
                //PRX_PRINT("Resolve Query: Path returned", PRX_TEXT_BROWN + IK_stage );
                return true;
            }
        }
        
        void motoman_planning_application_t::add_trajectory_to_database( space_point_t* start, config_t ee_config, trajectory_t& traj )
        {
            // PRX_PRINT( "Going to add trajectory!", PRX_TEXT_GREEN );
            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/prx_output/");

            std::string file = dir + "trajectories_" + _hand + ".database";

            PRX_PRINT(":: " << file, PRX_TEXT_LIGHTGRAY);

            std::ofstream traj_out;
            traj_out.open( file.c_str(), std::ofstream::app );

            // Output the start/goal pair
            traj_out << model->get_state_space()->serialize_point( start, 8 ) << "\n";
            traj_out << ee_config << "\n";

            //Then, output the trajectory itself
            traj.save_to_stream( traj_out );
            
            //Cleanup
            traj_out.close();
        }
        
    }
}
