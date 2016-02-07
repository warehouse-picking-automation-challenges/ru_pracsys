/**
 * @file simulated_grasp_sensor.hpp
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

#ifndef PRX_SIMULATED_TOUCH_SENSOR_HPP
#define	PRX_SIMULATED_TOUCH_SENSOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "simulation/sensing/grasp_sensor.hpp"
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
    }

    namespace packages
    {

        namespace manipulation
        {
            /**
             * @brief A sensor for detecting a single collision point on a specific system
             *
             * @author Andrew Dobson
             */
            class simulated_grasp_sensor_t : public grasp_sensor_t
            {
              public:
                simulated_grasp_sensor_t();
                virtual ~simulated_grasp_sensor_t();

                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                /**
                 * @copydoc grasp_sensor_t::update_data()
                 */
                virtual void update_data(); // updates the internal sensing data (communication)

              protected:
            };
        }
    }
}

#endif
