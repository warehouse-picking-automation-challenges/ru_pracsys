/**
 * @file grasp_sensor.hpp
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
#pragma once

#ifndef PRX_TOUCH_SENSOR_HPP
#define	PRX_TOUCH_SENSOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/sensing/sensor.hpp"
#include "prx/simulation/system_graph.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace sim
    {
        class collision_checker_t;
        class collision_list_t;
    }

    namespace packages
    {
        namespace baxter
        {
            class manipulator_plant_t;
        }

        namespace manipulation
        {
            class movable_body_plant_t;

            /**
             * @brief A sensor for detecting a single collision point on a specific system
             *
             * @author Andrew Dobson, Andrew Kimmel
             */
            class grasp_sensor_t : public sim::sensor_t
            {
            protected:
                sim::collision_checker_t* collision_checker;
                sim::collision_list_t* collision_list;
                sim::system_graph_t system_graph;

                std::vector<std::string> effectors_to_check;
                std::vector<std::string> movable_body_geom_names;
                std::vector< baxter::manipulator_plant_t* > manipulators;
                std::vector< movable_body_plant_t* > movable_bodies;

                bool single_object_detection;
                std::string single_object_name;

                std::vector<std::string> collided_systems;

            public:

                grasp_sensor_t();
                virtual ~grasp_sensor_t();

                /**
                 * Initializes from the given parameters.
                 *
                 * @brief Initializes from the given parameters.
                 *
                 * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
                 * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
                 * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
                 * is not specified in the \c reader then it will be read from the \c template_reader
                 */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);
                virtual void initialize_sensor(sim::simulator_t* sim);

                /**
                 * This function returns whether the specified system (system_path) is in collision.
                 * The name of the body colliding with system is returned by reference.  This function
                 * does not cover the case of multiple colliding points, as it will return whatever is
                 * stored first in the collision list.
                 *
                 */
                virtual void update_data() = 0; // updates the internal sensing data (communication)

                const std::vector<std::string>& get_collided_systems();

            };
        }
    }
}

#endif

