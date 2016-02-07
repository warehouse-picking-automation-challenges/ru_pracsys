/**
 * @file sensor.cpp 
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

#include "prx/simulation/sensing/sensor.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx
{
    using namespace util;
    
    namespace sim
    {

        pluginlib::ClassLoader<sensor_t> sensor_t::loader("prx_simulation", "prx::sim::sensor_t");
        
        sensor_t::sensor_t()
        {
        }

        sensor_t::~sensor_t()
        {
        }
        
        void sensor_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            
//            if( parameters::has_attribute("source", reader, template_reader) )
//            {
            source = parameters::get_attribute_as<std::string>("source", reader, template_reader, "no_source");
//            }
//            if( parameters::has_attribute("sensor_delay", reader, template_reader) )
//            {
            sensor_delay = parameters::get_attribute_as<double>("sensor_delay", reader, template_reader, -1);
            trigger_time = sensor_delay;
//            }
            periodic_sensing = parameters::get_attribute_as< bool >("periodic_sensing", reader, template_reader, true);
        }
        
        bool sensor_t::fires(double time_step)
        {
            if(!periodic_sensing)
            {
                return false;
            }
            trigger_time -= time_step;
            return (sensor_delay >= PRX_ZERO_CHECK && trigger_time <= PRX_ZERO_CHECK);
        }
        
        pluginlib::ClassLoader<sensor_t>& sensor_t::get_loader()
        {
            return loader;
        }
        
        std::string sensor_t::get_source()
        {
            return source;
        }
        
        void sensor_t::reset_delay()
        {
            trigger_time = sensor_delay;
        }
        
        void sensor_t::set_periodic_sensing(bool flag)
        {
            periodic_sensing = flag;
        }

        double sensor_t::get_sensor_delay()
        {
            return sensor_delay;
        }
        
        
        
    }
}

