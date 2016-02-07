/**
 * @file grasp_testing_planning_application.cpp 
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

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/applications/grasp_testing_planning_application.hpp"
#include "simulation/manipulator_simulator.hpp"
#include "prx/planning/world_model.hpp"

#include <pluginlib/class_list_macros.h>

 PLUGINLIB_EXPORT_CLASS( prx::plan::grasp_test_planning_application_t, prx::plan::planning_application_t )

 namespace prx
 {
    using namespace util;
    using namespace sim;
    using namespace prx::packages::manipulation;
    namespace plan
    {
        using namespace comm;
        
        grasp_test_planning_application_t::grasp_test_planning_application_t() 
        {
        }
        
        grasp_test_planning_application_t::~grasp_test_planning_application_t() 
        {   
        }

        bool grasp_test_planning_application_t::place_object_callback(prx_planning::place_object::Request& req, prx_planning::place_object::Response& res)
        {
            motoman_planning_application_t::place_object_callback( req, res );


            std::string context_name = _hand + "_" + req.object_name;
            std::string obstacle_name = "simulator/obstacles/" + req.object_name;
            geometry_msgs::PoseStamped pose = req.object_pose;

            for (int i = 0; i < 1; ++i)
            {

                PRX_INFO_S("Adding " << req.object_name << " to the planning node: " << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z);
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
                
                // sim::update_point_cloud = true;
                // model->update_sensing();
                // sim::update_point_cloud = false;

                //init_object_geometry(req.object_name);

                PRX_INFO_S("Done adding " << context_name << " to the planning node: ");

                res.success = true;


                config_t junk;
                state_t* st_junk = model->get_state_space()->alloc_point();

                consider_grasp.resize(500);

                std::string obj_name = req.object_name;


                // right hand: robotiq
                check_for_context_switch("right");
                for( unsigned i=0; i<consider_grasp.size(); ++i )
                {
                    consider_grasp[i] = true;
                }


                stop_updating_state();
                test_grasp_for_object( obj_name, st_junk, junk );
                start_updating_state();                

                // left hand: unigripper
                check_for_context_switch("left");
                for( unsigned i=0; i<consider_grasp.size(); ++i )
                {
                    consider_grasp[i] = true;
                }
                
                stop_updating_state();
                test_grasp_for_object( obj_name, st_junk, junk );
                start_updating_state();                
                
                model->get_state_space()->free_point(st_junk);
            }
        }

        // DEBUGGING STUFF
        bool grasp_test_planning_application_t::test_grasp_for_object( std::string object_to_grasp, state_t*& good_state , config_t& ee_config)
        {
            //Storage for the actual object pose
            config_t object_pose;
            std::vector< config_t > grasp_configurations;
            manipulator_plant_t* manipulator;

            //Alright, I am assuming I read in all the offset configurations during init()...
            //Figure out what object we are trying to grasp
            std::string object_name = object_to_grasp;

            //Then, figure out what its current pose is?
            //We also have a map from object name to its corresponding system: obstacle_list
            sim::system_ptr_t object = object_list[object_name];
            movable_body_plant_t* cast_object = dynamic_cast< movable_body_plant_t* >( object.get() );
            //Error checking: ensure that the requested thing is actually a movable body
            if( cast_object == NULL )
            {
                return false;
            }
            cast_object->get_configuration( object_pose );

            //Let's then create a list of end-effector configurations which are properly transformed

            //Get the relative orientations
            std::vector< config_t >* config_pointer = NULL;
            if( _hand == "left" )
            {
                config_pointer = &( unigripper_grasps[ object_name ] );
            }
            else
            {
                config_pointer = &( robotiq_grasps[ object_name ] );
            }
            PRX_ASSERT( config_pointer != NULL );
            const std::vector< config_t >& relative_configurations = *config_pointer;

            PRX_PRINT("THE FOLLOWING DATA IS FOR THE HAND: " << _hand, PRX_TEXT_RED);
            PRX_PRINT("Object has: [" << relative_configurations.size() << "] possible grasps.", PRX_TEXT_CYAN);

            config_t robotiq_config;
            robotiq_config.set_position( 0, 0.1, 0 );
            robotiq_config.set_orientation( 0.5, -0.5, -0.5, -0.5 );
            
            config_t uni_config;
            uni_config.set_position( 0.01, 0.0, 0.004 );
            uni_config.set_orientation( 0, 0, 0, 1 );
            
            grasp_configurations.resize( relative_configurations.size() );

            //Then, for each of those, put it into the global coordinate frame
            for( unsigned i=0; i<relative_configurations.size(); ++i )
            {
                if( consider_grasp[i] )
                {
                    if( _hand == "left" )
                    {
                        grasp_configurations[i] = uni_config;
                        grasp_configurations[i].relative_to_global( relative_configurations[i] );
                    }
                    else
                    {
                        grasp_configurations[i] = robotiq_config;
                        grasp_configurations[i].relative_to_global( relative_configurations[i] );
                    }

                    grasp_configurations[i].relative_to_global( object_pose );
                }
            }

            std::string object_pathname = cast_object->get_pathname() + "/body";
            PRX_PRINT("Object pathname to test: " << object_pathname, PRX_TEXT_MAGENTA);

            std::vector< std::string > manipulator_geoms;
            if( _hand == "right" )
            {
                manipulator_geoms.push_back( "simulator/a_right_manipulator/palm" );
                manipulator_geoms.push_back( "simulator/a_right_manipulator/finger_middle_link_2" );
                manipulator_geoms.push_back( "simulator/a_right_manipulator/finger_middle_link_3" );
                manipulator_geoms.push_back( "simulator/a_right_manipulator/finger_1_link_2" );
                manipulator_geoms.push_back( "simulator/a_right_manipulator/finger_1_link_3" );
                manipulator_geoms.push_back( "simulator/a_right_manipulator/finger_2_link_2" );
                manipulator_geoms.push_back( "simulator/a_right_manipulator/finger_2_link_3" );
            }
            else
            {
                manipulator_geoms.push_back( "simulator/a_left_manipulator/head_base" );
            }

            unsigned number_of_collisions = 0;
            unsigned number_of_ik_failures = 0;
            unsigned number_of_valid_states = 0;

            //Ask for an IK solution to that configuration
            if( _hand == "left" )
                manipulator = _left_manipulator;
            else
                manipulator = _right_manipulator;

            //DEBUGGING JUNK
            manipulation_simulator_t* manip_sim = dynamic_cast< manipulation_simulator_t* >( model->get_simulator() );
            unsigned num_graspable = 0;
            std::vector< unsigned > grasp_misses;
            std::vector< unsigned > grasp_collisions;
            std::vector< unsigned > valid_grasps;

            bool good_ik = false;
            //Then, for each of these configurations
            for( unsigned i=0; i<grasp_configurations.size(); ++i )
            {
                if( consider_grasp[i] )
                {
                // PRX_PRINT("Testing: [" << i << "]", PRX_TEXT_LIGHTGRAY);
                    good_ik = test_simple_IK( grasp_configurations[i], good_state, IK_plan );

                    // PRX_PRINT( manipulator->get_state_space()->print_point( good_state, 4 ), PRX_TEXT_LIGHTGRAY );

                // PRX_PRINT("good_state: " << good_state, PRX_TEXT_MAGENTA);
                // sleep(100);

                //If we get a solution, collision-check it.
                    if( good_ik )
                    {
                        bool good_grasp = true;

                    // PRX_PRINT("Got a good IK!", PRX_TEXT_GREEN);
                        good_state->at(16) = 1.0;
                        ee_config = grasp_configurations[i];

                        _left_manipulator->get_state_space()->copy_from_point( good_state );
                        _right_manipulator->get_state_space()->copy_from_point( good_state );
                        model->get_active_space()->copy_from_point(object_start_state);

                    //Testing for the virtual body to see if we can grasp the object here
                        if( manip_sim->can_grasp_object() )
                        {
                            ++num_graspable;
                            PRX_PRINT("grasp id: " << i, PRX_TEXT_MAGENTA);
                            // tf_broadcasting();
                            // // sleep(10);
                            // if(i == 1)
                            // {   
                            //     sleep(30);
                            // }
                        }
                        else
                        {
                            grasp_misses.push_back( i );
                            good_grasp = false;
                            PRX_PRINT("missed!!! ", PRX_TEXT_RED);
                            tf_broadcasting();
                            // sleep(5);
                        }

                    //Check if there is a collision between the hand and the object here
                        collision_list_t* clist = model->get_colliding_bodies( );
                        bool hand_object_collision = false;
                        foreach( collision_pair_t p, clist->get_body_pairs() )
                        {
                            if( p.first == object_pathname )
                            {
                                for( unsigned k=0; k<manipulator_geoms.size() && !hand_object_collision; ++k )
                                {
                                    if( p.second == manipulator_geoms[k] )
                                    {
                                        hand_object_collision = true;
                                    }
                                }
                            }
                            else if( p.second == object_pathname )
                            {
                                for( unsigned k=0; k<manipulator_geoms.size() && !hand_object_collision; ++k )
                                {
                                    if( p.first == manipulator_geoms[k] )
                                    {
                                        hand_object_collision = true;
                                    }
                                }
                            }
                        }
                        if( hand_object_collision )
                        {
                            grasp_collisions.push_back( i );
                            good_grasp = false;

                            PRX_PRINT("collision!!! ", PRX_TEXT_RED);
                            tf_broadcasting();
                            sleep(5);

                            // sleep(3);

                            

                        }

                        if( good_grasp )
                        {
                            valid_grasps.push_back( i );
                            // tf_broadcasting();
                            // sleep(5);
                        }
                    }
                }
            }

            PRX_PRINT("Percent good grasps: " << ((double)num_graspable)/((double)number_of_valid_states), PRX_TEXT_GREEN );
            
            PRX_PRINT("Valid Grasps: ", PRX_TEXT_GREEN);
            for( unsigned i=0; i<valid_grasps.size(); ++i )
            {
                PRX_PRINT(":: " << valid_grasps[i], PRX_TEXT_LIGHTGRAY);
                consider_grasp[ valid_grasps[i] ] = false;
            }
            PRX_PRINT("Grasp Misses: ", PRX_TEXT_MAGENTA);
            for( unsigned i=0; i<grasp_misses.size(); ++i )
            {
                PRX_PRINT(":: " << grasp_misses[i], PRX_TEXT_LIGHTGRAY);
                grasps_to_remove.push_back( grasp_misses[i] );
                consider_grasp[ grasp_misses[i] ] = false;
            }
            PRX_PRINT("Grasp Collisions: ", PRX_TEXT_RED);
            for( unsigned i=0; i<grasp_collisions.size(); ++i )
            {
                PRX_PRINT(":: " << grasp_collisions[i], PRX_TEXT_LIGHTGRAY);
                grasps_to_remove.push_back( grasp_collisions[i] );
                consider_grasp[ grasp_collisions[i] ] = false;
            }

            return false;
        }


        bool grasp_test_planning_application_t::test_simple_IK( const config_t& ee_config, space_point_t* target_state, plan_t& resulting_plan )
        {
            if( IK_seeds[0] )
                model->get_state_space()->free_point( IK_seeds[0] );
            IK_seeds[0] = model->get_state_space()->alloc_point();
            model->get_state_space()->copy_point( IK_seeds[0], current_state );
            impose_hand_state( IK_seeds[0] );

            for( int i=1; i<IK_seeds.size(); i++ )
            {
                if( IK_seeds[i] )
                    model->get_state_space()->free_point( IK_seeds[i] );
                IK_seeds[i] = model->get_state_space()->alloc_point();
                model->get_state_space()->uniform_sample( IK_seeds[i] );
                impose_hand_state( IK_seeds[i] );
            }

            manipulator_plant_t* manipulator = ( _hand == "left" ? _left_manipulator : _right_manipulator );

            unsigned number_of_valid_states = 0;
            //Then, for each seed from the database
            for( unsigned i=0; i<IK_seeds.size(); ++i )
            {
                bool success = false;
                success = manipulator->IK_solver( ee_config, target_state, _grasping, IK_seeds[i], true );

                if( success )
                {
                    return true;
                }
                if( _grasping )
                {
                    model->get_active_space()->copy_from_point(object_start_state);
                }
            }

            //If there are no valid free states, abort out, no need to do more work
            if( number_of_valid_states == 0 )
            {
                //              PRX_PRINT( "NO SEED WORKED :( out of " << IK_seeds.size(), PRX_TEXT_RED );
                return false;
            }

            return true;
        }        
    }
}
