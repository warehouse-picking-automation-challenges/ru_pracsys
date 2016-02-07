/**
 * @file grasp_sensor.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "simulation/sensing/grasp_sensor.hpp"
#include "simulation/plants/movable_body_plant.hpp"
#include "../../../baxter/simulation/plants/manipulator.hpp"

#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {

        namespace manipulation
        {
            grasp_sensor_t::grasp_sensor_t()
            {
                collision_checker = NULL;
                collision_list = NULL;
                single_object_name = "";
            }

            grasp_sensor_t::~grasp_sensor_t()
            {
                if( collision_list != NULL )
                {
                    delete collision_list;
                }
            }

            void grasp_sensor_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                sensor_t::init(reader, template_reader);
                //Check to see if we are caring only about a single object
                if( parameters::has_attribute("single_object", reader, template_reader) )
                {
                    single_object_name = parameters::get_attribute("single_object", reader, template_reader);
                    single_object_detection = true;
                }
            }

            void grasp_sensor_t::initialize_sensor(simulator_t* sim)
            {
                //Begin by getting all of the plants in the simulator
                sim->update_system_graph(system_graph);
                std::vector<plant_t*> plants;
                system_graph.get_plants( plants );

                //Allocate the collision list we will use for pair checking
                collision_list = new vector_collision_list_t;

                //For each plant in the simulator
                foreach(plant_t* sys, plants)
                {
                    //First, check if the system is a manipulator
                    baxter::manipulator_plant_t* manip = dynamic_cast<baxter::manipulator_plant_t*>(sys);
                    if( manip != NULL )
                    {
                        // PRX_INFO_S("adding manipulator to sensor: "<<manip);
                        //Remember some information about these manipulators
                        manipulators.push_back( manip );
                        //Including end-effector names
                        effectors_to_check.push_back(manip->get_end_effector_name());
                    }
                    //If it is not a manipulator
                    else
                    {
                        //Then, see if it is a movable body
                        movable_body_plant_t* body = dynamic_cast<movable_body_plant_t*>(sys);
                        if( body != NULL )
                        {
                            // PRX_INFO_S("adding body to sensor: "<<body);
                            bool add_body = true;
                            //Then, if we are doing single object detection
                            if( single_object_detection )
                            {
                                //Check if this body is the one we are interested in
                                if( body->get_pathname() != single_object_name )
                                {
                                    //If it is not, we will not add the body
                                    add_body = false;
                                }
                            }
                            //If we are remembering this body
                            if( add_body )
                            {
                                //Remember the body
                                movable_bodies.push_back( body );
                                //Also need to get the root geom name for the body
                                movable_body_geom_names.push_back( body->get_root_geom_name() );
                            }
                        }
                    }
                }

                //Now, build up the collision list we will use to check things
                //For each manipulator
                for( unsigned i=0; i<effectors_to_check.size(); ++i )
                {
                    //For each movable body
                    for( unsigned j=0; j<movable_body_geom_names.size(); ++j )
                    {
                        //Add this pair to the collision list
                        collision_list->add_pair( effectors_to_check[i], movable_body_geom_names[j] );
                    }
                }

                //Also, we will have a list of strings we can return
                collided_systems.resize(effectors_to_check.size());
                //Store the collision checker to do our things
                collision_checker = sim->get_collision_checker();
                PRX_ASSERT( collision_checker != NULL );
            }

            const std::vector<std::string>& grasp_sensor_t::get_collided_systems()
            {
                return collided_systems;
            }
        }
    }
}

