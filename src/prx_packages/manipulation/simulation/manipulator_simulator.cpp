/**
 * @file simulator.hpp
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

#include "simulation/manipulator_simulator.hpp"
#include "simulation/sensing/grasp_sensing_info.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/cost_functions/default_uniform.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp> // boost::tie

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_simulator_t, prx::sim::simulator_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {

        namespace manipulation
        {

            manipulation_simulator_t::manipulation_simulator_t() : null_response_simulator_t()
            {
                grasp_sensing_info = NULL;

                check_states = false;
                check_poses = false;

                updated_collision_list = new vector_collision_list_t();
            }

            manipulation_simulator_t::~manipulation_simulator_t()
            {
            }

            void manipulation_simulator_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                null_response_simulator_t::init(reader, template_reader);

                grasp_sensor_name = parameters::get_attribute_as<std::string > ("grasp_sensor_source", reader, template_reader);
                PRX_DEBUG_COLOR("Got grasp sensor name: " << grasp_sensor_name, PRX_TEXT_MAGENTA);
                if( parameters::has_attribute("manipulator_names", reader, template_reader) )
                {
                    foreach(const parameter_reader_t* list_reader, parameters::get_list("manipulator_names", reader, template_reader))
                    {
                        std::string tmp_string = list_reader->get_attribute("");
                        manipulators.push_back(dynamic_cast<manipulator_plant_t*>( plants[tmp_string]) );
                        PRX_ASSERT(manipulators.back() != NULL);
                        //Remember the name of the end-effector geometry for this manipulator
                        effector_names.push_back(manipulators.back()->get_end_effector_name());
                        ignore_during_grasping.push_back(manipulators.back()->get_ignored_grasping_bodies());

                        //Also begin by assuming this manipulator grasps no object
                        grasped_objects.push_back(NULL);
                    }
                    relative_configs.resize( manipulators.size() );
                }
                else
                {
                    PRX_FATAL_S("No manipulator name list specified for manipulator simulator.");
                }

                //If there are states for the manipulator we wish to check, we read them in here
                if( parameters::has_attribute("check_states", reader, template_reader) )
                {
                    foreach(const parameter_reader_t* s_reader, parameters::get_list("check_states", reader, template_reader))
                    {
                        states_to_check.push_back(s_reader->get_attribute_as<std::vector<double> >("state"));
                    }
                    check_state_pos = 0;
                    check_states = true;
                }
                //If there are object poses we wish to check, read them in here.
                if( parameters::has_attribute("check_poses", reader, template_reader) )
                {
                    foreach(const parameter_reader_t* s_reader, parameters::get_list("check_poses", reader, template_reader))
                    {
                        poses_to_check.push_back(s_reader->get_attribute_as<std::vector<double> >("pose"));
                    }
                    check_poses_pos = 0;
                    check_poses = true;
                }

                if ( parameters::has_attribute("manipulation_sensing_info", reader, template_reader))
                {
                    PRX_DEBUG_COLOR ("Controller : " << pathname << " has sensing info defined.", PRX_TEXT_MAGENTA);
       
                    sensing_info_t* new_info = parameters::create_from_loader<sensing_info_t > ("prx_simulation", reader, "manipulation_sensing_info", template_reader, "manipulation_sensing_info");
                    new_info->link_active(&active);
                    PRX_DEBUG_COLOR("BEFORE initializing sensing info...", PRX_TEXT_RED);
                    parameters::initialize(new_info, reader, "manipulation_sensing_info", template_reader, "manipulation_sensing_info");
                    PRX_DEBUG_COLOR("AFTER initializing sensing info...", PRX_TEXT_RED);

                    sensing_info.push_back(new_info);
                }
            }

            void manipulation_simulator_t::initialize_sensing()
            {
                simulator_t::initialize_sensing();
                //Search over all the sensing infos
                foreach(sensing_info_t* info, sensing_info)
                {
                    //And try to find the grasp sensing info
                    grasp_sensing_info = dynamic_cast<grasp_sensing_info_t*>(info);
                    if( grasp_sensing_info != NULL )
                    {
                        //If we found it, make sure it is not sensing periodically
                        grasp_sensing_info->set_periodic_sensing(grasp_sensor_name, false);
                        return;
                    }
                }
                PRX_FATAL_S("Could not find a grasp sensing info.  Aborting.");
            }

            void manipulation_simulator_t::link_collision_list(collision_list_t* collision_list)
            {
                updated_collision_list->clear();
                collision_pairs.clear();

                PRX_PRINT("Number of collision pairs: "<<collision_list->size(),PRX_TEXT_RED);
                foreach(collision_pair_t p, collision_list->get_body_pairs())
                {
                    bool found_ee = false;
                    for( unsigned i = 0; i < effector_names.size() && !found_ee; ++i )
                    {
                        if( p.first == effector_names[i] || p.second == effector_names[i] )
                        {
                            found_ee = true;
                        }
                    }
                    if(!found_ee)
                        updated_collision_list->add_new_pair(p);
                }
                PRX_PRINT("Number of collision pairs: "<<updated_collision_list->size(),PRX_TEXT_RED);

                collision_checker->link_collision_list(updated_collision_list);
            }

            bool manipulation_simulator_t::can_grasp_object()
            {
                //Update the state
                compute_relative_config( );                

                // PRX_PRINT("Testing state: " << state_space->print_memory( 3 ), PRX_TEXT_GREEN);
                for( unsigned i=0; i<grasped_names.size(); ++i )
                    if(!(grasped_names[i].empty()))
                        return true;
                return false;       
            }

            bool manipulation_simulator_t::in_collision()
            {
//                PRX_INFO_S("The in_collision function of the manipulation simulator is called");

                update_ground_truth();
                return null_response_simulator_t::in_collision();

/*
                sim::collision_list_t* in_collision_list = null_response_simulator_t::get_colliding_bodies();

                foreach( collision_pair_t p, in_collision_list->get_body_pairs() )
                {
                    bool valid_collision = true;
                    for( unsigned i = 0; valid_collision == true && i < effector_names.size(); ++i )
                    {
                        if( p.first == effector_names[i] || p.second == effector_names[i] )
                            valid_collision = false;
                        else if( manipulators[i]->is_grasping() )
                        {
                            for( unsigned j=0; valid_collision==true && j < ignore_during_grasping[i].size(); ++j )
                            {
                                if( p.first == ignore_during_grasping[i][j] || p.second == ignore_during_grasping[i][j] )
                                    valid_collision = false;
                            }
                        }
                    }

                    if( valid_collision )
                    {
                        return true;
                    }
                }
                return false;
*/
            }

            void manipulation_simulator_t::push_state(const sim::state_t * const state)
            {
                //Update the state
                null_response_simulator_t::push_state(state);
                update_ground_truth();
            }

            void manipulation_simulator_t::print_relative_config()
            {
                for( unsigned i=0; i<grasped_names.size(); ++i )
                {
                    const std::string& grasped_name = grasped_names[i];

//                    PRX_PRINT( "Inside printing for index " << i, PRX_TEXT_GREEN );
                    if( !grasped_name.empty() && manipulators[i]->is_grasping() )
                    {
//                        PRX_PRINT( "The manipulator " << i << " is grasping", PRX_TEXT_GREEN );
                        if( grasped_objects[i] != NULL && grasped_objects[i]->is_active() )
                        {
                            PRX_PRINT( "The relative configuration is: " << relative_configs[i], PRX_TEXT_GREEN );
                        }
/*                        else
                        {
                            PRX_PRINT( "There is no grasped object", PRX_TEXT_GREEN );
                        }
*/                    }
/*                    else
                    {
                        PRX_PRINT( "There is no grasped object for " << i, PRX_TEXT_GREEN );
                    }
*/                }
            }

            void manipulation_simulator_t::compute_relative_config( )
            {
                grasp_sensing_info->forced_sense(grasp_sensor_name);
                grasped_names = grasp_sensing_info->get_collided_systems();

                //For each manipulator
                for( unsigned i=0; i<grasped_names.size(); ++i )
                {
                    const std::string& grasped_name = grasped_names[i];

                    if( !grasped_name.empty() && manipulators[i]->is_grasping() )
                    {
                        grasped_objects[i] = dynamic_cast< movable_body_plant_t* >(plants[grasped_name]);
                        if( grasped_objects[i] != NULL && grasped_objects[i]->is_active() )
                        {
                            manipulators[i]->get_end_effector_configuration(effector_config);
                            config_t object_config;
                            grasped_objects[i]->get_configuration( object_config );
                            //PRX_PRINT( "From manip_config " << effector_config << " and object_config " << object_config, PRX_TEXT_LIGHTGRAY );
                            grasped_objects[i]->relative_config_with_manipulator( effector_config, relative_configs[i] );
                            //PRX_PRINT( "Computed relative configuration: " << relative_configs[i], PRX_TEXT_LIGHTGRAY );
                        }
                    }
                    else
                        grasped_objects[i] = NULL;
                }
            }


            void manipulation_simulator_t::propagate(const double simulation_step)
            {
                //Thanasi's special checks
                if( check_states || check_poses )
                {
                    state_checking_propagate();
                    return;
                }

                //Do the standard propagation
                simulator_t::propagate( simulation_step );

                update_ground_truth();
            }

            void manipulation_simulator_t::update_ground_truth()
            {
                //For each manipulator
                for( unsigned i=0; i<manipulators.size(); ++i )
                {
                    //If this manipulator is manipulating an object
                    if( grasped_objects[i] != NULL )
                    {
                        //Get the end effector position
                        manipulators[i]->get_end_effector_configuration(effector_config);
                        //Then move the object
                        grasped_objects[i]->move_object( effector_config, relative_configs[i] );
                    }
                }
            }

            void manipulation_simulator_t::propagate_and_respond()
            {
                propagate(simulation::simulation_step);
                in_collision();
            }

            bool manipulation_simulator_t::internal_state_push()
            {
                return true;
            }

            void manipulation_simulator_t::state_checking_propagate()
            {
                if( check_states )
                {
                    if( check_state_pos >= states_to_check.size() )
                        check_state_pos = 0;

                    const space_t* manip_space = manipulators[0]->get_state_space();
                    state_t* state = manip_space->alloc_point();
                    std::vector<double> manip_vec(manip_space->get_dimension());
                    for( unsigned i = 0; i < manip_space->get_dimension(); ++i )
                    {
                        manip_vec[i] = states_to_check[check_state_pos][i];
                    }
                    manip_space->copy_vector_to_point(manip_vec, state);
                    manip_space->copy_from_point(state);
                    check_state_pos++;
                    PRX_DEBUG_COLOR("Manip: " << manip_space->print_memory(5), PRX_TEXT_CYAN);
                    manip_space->free_point(state);
                }

                if( check_poses )
                {
                    movable_body_plant_t* object = NULL;
                    for( subsystems_iter = subsystems.begin(); subsystems_iter != subsystems.end() && object == NULL; ++subsystems_iter )
                    {
                        if( object == NULL && dynamic_cast<movable_body_plant_t*>(subsystems_iter->second.get()) != NULL )
                            object = static_cast<movable_body_plant_t*>(subsystems_iter->second.get());
                    }

                    if( check_poses_pos >= poses_to_check.size() )
                        check_poses_pos = 0;

                    const space_t* object_space = object->get_state_space();
                    state_t* state = object_space->alloc_point();
                    std::vector<double> object_vec(object_space->get_dimension());

                    for( unsigned i = 0; i < object_space->get_dimension(); ++i )
                    {
                        object_vec[i] = poses_to_check[check_poses_pos][i];
                    }

                    object_space->copy_vector_to_point(object_vec, state);
                    object_space->copy_from_point(state);
                    PRX_DEBUG_COLOR("Pose : " << object_space->print_memory(5), PRX_TEXT_GREEN);
                    check_poses_pos++;
                    object_space->free_point(state);
                }
            }

        }
    }
}

