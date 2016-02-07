/**
 * @file grasp_sensing_info.hpp 
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
#pragma once

#ifndef PRX_GRASP_SENSING_INFO_HPP
#define	PRX_GRASP_SENSING_INFO_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/sensing/sensing_info.hpp"
#include "prx/simulation/system_ptr.hpp"

namespace prx
{
    namespace util
    {
        class geometry_t;
    }
    namespace packages
    {
        namespace manipulation
        {
            class grasp_sensor_t;
        
            /**
             * 
             * @author Andrew Dobson
             */
            class grasp_sensing_info_t : public sim::sensing_info_t
            {
              protected:
                grasp_sensor_t* grasp_sensor;

                std::vector<bool> check_collision_flags;
                //std::vector<std::string> collided_systems;

              public:
                grasp_sensing_info_t();
                virtual ~grasp_sensing_info_t();
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                virtual void set_sensors(const std::vector<sim::sensor_t*>& sensors);
                // The conversion function to turn sensing data into the controller's info
                virtual void update_info();
                // Used to access the info?
                virtual void get_info();

                virtual const std::vector<std::string>& get_collided_systems();
            };
        }
    }
}

#endif

