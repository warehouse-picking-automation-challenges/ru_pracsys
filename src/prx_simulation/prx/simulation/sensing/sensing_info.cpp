/**
 * @file sensing_info.cpp 
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

#include "prx/simulation/sensing/sensing_info.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/systems/system.hpp"

namespace prx
{
    using namespace util;
    
    namespace sim
    {

        pluginlib::ClassLoader<sensing_info_t> sensing_info_t::loader("prx_simulation", "prx::sim::sensing_info_t");
        
        sensing_info_t::sensing_info_t() 
        {
            linked_active_flag = NULL;
        }

        sensing_info_t::~sensing_info_t()
        {
        }
        
        void sensing_info_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            PRX_DEBUG_COLOR("Sensing info init", PRX_TEXT_BLUE);
            if( parameters::has_attribute("sources", reader, template_reader) )
            {
                std::vector<const parameter_reader_t*> sources = parameters::get_list("sources", reader, template_reader);
                foreach(const parameter_reader_t* source_reader, sources)
                {
                    std::string new_source = source_reader->get_attribute("");
                    sensor_sources.push_back(new_source);
                    PRX_DEBUG_COLOR("Read in sensor source: " << new_source, PRX_TEXT_CYAN);
                }
            }
            else
            {
                PRX_ERROR_S("Sensing info defined without any sensor sources!");
            }
            if( parameters::has_attribute("update_delay", reader, template_reader) )
            {
                update_delay = parameters::get_attribute_as<double>("update_delay", reader, template_reader);
            }
            else
            {
                PRX_ERROR_S ("No update delay specified. Will set to sim_step");
                update_delay = simulation::simulation_step;
            }
            
            trigger_time = update_delay;
            PRX_ASSERT(linked_active_flag != NULL);
        
        }
        
        void sensing_info_t::link_active(bool* active_flag)
        {
            linked_active_flag = active_flag;
        }
        bool sensing_info_t::is_active()
        {
            return (*linked_active_flag);
        }
        
        void sensing_info_t::update_info()
        {
            PRX_WARN_S ("Update info not implemented in base class!");
        }
        
        void sensing_info_t::get_info()
        {
            PRX_WARN_S ("Get info not implemented in base class!");
        }
        
        void sensing_info_t::set_sensors(const std::vector<sensor_t*>& sensors)
        {
            linked_sensors = sensors;
        }
        
        std::vector<std::string> sensing_info_t::get_sensor_sources()
        {
            return sensor_sources;
        }
        
        bool sensing_info_t::updates(double time_step)
        {
            trigger_time -= time_step;
            return (update_delay >= PRX_ZERO_CHECK && trigger_time <= PRX_ZERO_CHECK);
        }
        
        void sensing_info_t::reset_delay()
        {
            trigger_time = update_delay;
        }
        
        void sensing_info_t::set_periodic_sensing(std::string sensor_name, bool flag)
        {
            bool found = false;
            for( unsigned i=0; i<sensor_sources.size(); ++i)
            {
                if(sensor_sources[i] == sensor_name)
                {
                    linked_sensors[i]->set_periodic_sensing(flag);
                    found = true;
                    break;
                }
            }
            if(!found)
            {
                PRX_WARN_S("Sensor \"" << sensor_name << "\" not found in sensing info.");
            }
        }
        
        void sensing_info_t::forced_sense(std::string sensor_name)
        {
            //PRX_DEBUG_COLOR("Beginning forced sense....", PRX_TEXT_MAGENTA);
            bool found = false;
            for( unsigned i=0; i<sensor_sources.size(); ++i)
            {
                //PRX_DEBUG_COLOR("Checking source: " << i << " : " << sensor_sources[i], PRX_TEXT_CYAN);
                if(sensor_sources[i] == sensor_name)
                {
                    linked_sensors[i]->update_data();
                    found = true;
                    break;
                }
            }
            if(!found)
            {
                PRX_WARN_S("Sensor \"" << sensor_name << "\" not found in sensing info.");
            }
            //PRX_DEBUG_COLOR("Finished forced sense....", PRX_TEXT_MAGENTA);
        }

        
        pluginlib::ClassLoader<sensing_info_t>& sensing_info_t::get_loader()
        {
            return loader;
        }
    }
}

