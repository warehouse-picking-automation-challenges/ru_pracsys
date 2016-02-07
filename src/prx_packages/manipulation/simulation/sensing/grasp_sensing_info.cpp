/**
 * @file grasp_sensing_info.cpp
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

#include "simulation/sensing/grasp_sensing_info.hpp"
#include "simulation/sensing/grasp_sensor.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
//#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasp_sensing_info_t, prx::sim::sensing_info_t);

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace manipulation
        {
            grasp_sensing_info_t::grasp_sensing_info_t()
            {
                grasp_sensor = NULL;
            }

            grasp_sensing_info_t::~grasp_sensing_info_t()
            {
            }

            void grasp_sensing_info_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
            {
                sensing_info_t::init(reader, template_reader);
            }

            void grasp_sensing_info_t::set_sensors(const std::vector<sim::sensor_t*>& sensors)
            {
                sensing_info_t::set_sensors(sensors);
                /** Find the relevant sensors */
                foreach(sensor_t* sensor, sensors)
                {
                    // PRX_DEBUG_COLOR("\n\n = GOT A SENSOR! = \n\n", PRX_TEXT_CYAN);
                    if (dynamic_cast<grasp_sensor_t*>(sensor) != NULL)
                        grasp_sensor = dynamic_cast<grasp_sensor_t*>(sensor);
                }
                PRX_ASSERT(grasp_sensor != NULL);
            }

            void grasp_sensing_info_t::update_info()
            {
                //Nothing?
            }

            void grasp_sensing_info_t::get_info()
            {
                PRX_WARN_S("Get info not implemented for grasp sensing info.");
            }

            const std::vector<std::string>& grasp_sensing_info_t::get_collided_systems()
            {
                return grasp_sensor->get_collided_systems();
            }
        }
    }
}
