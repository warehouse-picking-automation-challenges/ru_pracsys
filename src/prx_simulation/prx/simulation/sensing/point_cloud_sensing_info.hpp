/**
 * @file point_cloud_sensing_info.hpp 
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

#ifndef PRX_POINT_CLOUD_SENSING_INFO_HPP
#define PRX_POINT_CLOUD_SENSING_INFO_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/sensing/sensing_info.hpp"
#include "prx/simulation/system_ptr.hpp"

namespace prx
{
	namespace util
	{

	}
	namespace sim
	{

        class point_cloud_sensor_t;
    
        /**
         * 
         * @author Zakary Littlefield
         */
        class point_cloud_sensing_info_t : public sensing_info_t
        {
          protected:
            point_cloud_sensor_t* point_cloud_sensor;

          public:
            point_cloud_sensing_info_t();
            virtual ~point_cloud_sensing_info_t();
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            virtual void set_sensors(const std::vector<sensor_t*>& sensors);
            // The conversion function to turn sensing data into the controller's info
            virtual void update_info();
            // Used to access the info?
            virtual void get_info();
        };
	}
}


#endif