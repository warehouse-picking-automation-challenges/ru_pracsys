/**
 * @file simulated_grasp_sensor.cpp
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

#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "../../../baxter/simulation/plants/manipulator.hpp"

#include "simulation/plants/movable_body_plant.hpp"
#include "simulation/sensing/simulated_grasp_sensor.hpp"
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::simulated_grasp_sensor_t, prx::sim::sensor_t);

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {

        namespace manipulation
        {

            simulated_grasp_sensor_t::simulated_grasp_sensor_t()
            {
                collision_checker = NULL;
                single_object_detection = false;
            }

            simulated_grasp_sensor_t::~simulated_grasp_sensor_t() { }

            void simulated_grasp_sensor_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
            {
                grasp_sensor_t::init(reader, template_reader);
            }

            void simulated_grasp_sensor_t::update_data()
            {
                config_t tmp_config;
                // PRX_PRINT("Sening update data", PRX_TEXT_BROWN);
                //First, we need to update all of the geometries we care about
                for( unsigned i=0; i<manipulators.size(); ++i )
                {
                    //Get the manipulator's end effector configuration
                    manipulators[i]->get_end_effector_configuration( tmp_config );
                    tmp_config.normalize_orientation();

                    collision_checker->set_configuration( effectors_to_check[i], tmp_config );
                    // PRX_PRINT("Effector [" << effectors_to_check[i] << "]: " << tmp_config, PRX_TEXT_CYAN);
                }
                for( unsigned i=0; i<movable_bodies.size(); ++i )
                {
                    //Get the manipulator's end effector configuration
                    movable_bodies[i]->get_configuration( tmp_config );
                    tmp_config.normalize_orientation();

                    collision_checker->set_configuration( movable_body_geom_names[i], tmp_config );
                    // PRX_PRINT("Body [" << movable_body_geom_names[i] << "]: " << tmp_config, PRX_TEXT_GREEN);
                }

                //Get the colliding bodies according to the collision list
                sim::collision_list_t* in_collision_list = collision_checker->colliding_bodies( collision_list );

                //Then, begin by assuming that there are no grasps detected
                for( unsigned i=0; i<effectors_to_check.size(); ++i )
                    collided_systems[i] = "";

                // bool collision_with_end_effector = false;
                for( unsigned i = 0; i < effectors_to_check.size() ; ++i )
                {
                    if(manipulators[i]->is_grasping())
                    {
                        bool collision_with_end_effector = false;
                        foreach(collision_pair_t p, in_collision_list->get_body_pairs())
                        {
                            if(!collision_with_end_effector)
                            {
                                //Because the collision list only has pairs we care about, one or the other HAS to be an end-effector
                                if( p.first == effectors_to_check[i] )
                                {
                                    collision_with_end_effector = true;
                                    collided_systems[i] = p.second;
                                }
                                else if( p.second == effectors_to_check[i] )
                                {
                                    collision_with_end_effector = true;
                                    collided_systems[i] = p.first;
                                }
                            }
                        }
                    }
                }

                //Strip away the pathnames to get just the system name
                std::string subpath;
                for( unsigned i=0; i<collided_systems.size(); ++i )
                    if( !collided_systems[i].empty() )
                        boost::tie(collided_systems[i], subpath) = reverse_split_path( collided_systems[i] );
            }
        }
    }
}

