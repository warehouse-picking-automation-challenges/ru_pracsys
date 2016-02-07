/**
 * @file point_cloud_sensing_info.cpp 
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

#include "prx/simulation/sensing/point_cloud_sensing_info.hpp"
#include "prx/simulation/sensing/sensors/point_cloud_sensor.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"


#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
//#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS(prx::sim::point_cloud_sensing_info_t, prx::sim::sensing_info_t);

namespace prx
{
	using namespace util;
	namespace sim
	{
        point_cloud_sensing_info_t::point_cloud_sensing_info_t()
        {
            point_cloud_sensor = NULL;
        }

        point_cloud_sensing_info_t::~point_cloud_sensing_info_t()
        {
        }

        void point_cloud_sensing_info_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            sensing_info_t::init(reader, template_reader);
        }

        void point_cloud_sensing_info_t::set_sensors(const std::vector<sim::sensor_t*>& sensors)
        {
            sensing_info_t::set_sensors(sensors);
            /** Find the relevant sensors */
            foreach(sensor_t* sensor, sensors)
            {
                PRX_DEBUG_COLOR("\n\n = GOT A SENSOR! = \n\n", PRX_TEXT_CYAN);
                if (dynamic_cast<point_cloud_sensor_t*>(sensor) != NULL)
                    point_cloud_sensor = dynamic_cast<point_cloud_sensor_t*>(sensor);
            }
            PRX_ASSERT(point_cloud_sensor != NULL);
        }

        void point_cloud_sensing_info_t::update_info()
        {
            point_cloud_sensor->update_data();
        }

        void point_cloud_sensing_info_t::get_info()
        {
            PRX_WARN_S("Get info not implemented for point cloud sensing info.");
        }
	}
}