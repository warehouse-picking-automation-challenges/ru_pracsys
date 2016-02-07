/**
 * @file velocity_sensor.cpp 
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

#include "prx/simulation/sensing/sensors/velocity_sensor.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::velocity_sensor_t, prx::sim::sensor_t);

namespace prx
{
    using namespace util;
    
    namespace sim
    {
        
        velocity_sensor_t::velocity_sensor_t()
        {
        }

        velocity_sensor_t::~velocity_sensor_t()
        {
        }
        
        void velocity_sensor_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            config_sensor_t::init(reader, template_reader);
        
        }
        
        void velocity_sensor_t::update_data()
        {
           // PRX_DEBUG_S ("Updating velocities!");
            previous_plant_configs = plant_configs;
            config_sensor_t::update_data();
            
            if (previous_plant_configs.size() != plant_configs.size())
            {
              
                PRX_WARN_S ("Cannot calculate velocities - differently sized config lists");
                for(unsigned i = 0; i < plant_configs.size(); i++)
                {
                    
                    // TODO: Does each sensed plant need an individual delay associated with it?
                    velocities[plant_configs[i].first] = plant_configs[i].second.get_position();
                    velocities[plant_configs[i].first].zero();
                    
//                    PRX_DEBUG_S("plant config at i:" << plant_configs[i].first);
                }
            }
            else
            {
                for(unsigned i = 0; i < plant_configs.size(); i++)
                {
                    // TODO: Does each sensed plant need an individual delay associated with it?
                    velocities[plant_configs[i].first] = ( (plant_configs[i].second - previous_plant_configs[i].second).get_position()/sensor_delay);
//                    PRX_DEBUG_COLOR ("Velocity for : " << plant_configs[i].first << " is: " << velocities[plant_configs[i].first], PRX_TEXT_GREEN);
                }
            }
        }
        
        hash_t<std::string, vector_t> velocity_sensor_t::get_velocities()
        {
            return velocities;
        }
        
    }
}