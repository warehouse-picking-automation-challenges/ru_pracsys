/**
 * @file sensing_model.cpp 
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

#include "prx/simulation/sensing/sensing_model.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS(prx::sim::sensing_model_t, prx::sim::sensing_model_t);
namespace prx
{
    using namespace util;
    
    namespace sim
    {
        pluginlib::ClassLoader<sensing_model_t> sensing_model_t::loader("prx_simulation", "prx::sim::sensing_model_t");
        
        sensing_model_t::sensing_model_t() 
        {
            sensed_state = NULL;
            state_space = NULL;
        }

        sensing_model_t::~sensing_model_t() 
        {
            if (state_space != NULL)
                state_space->free_point(sensed_state);
            foreach(sensor_t* sensor, all_sensors)
            {
                delete sensor;
            }
        }
        
        void sensing_model_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            std::string template_name;
            if( parameters::has_attribute("sensors", reader, template_reader) )
            {
                parameter_reader_t::reader_map_t sensors_map = parameters::get_map("sensors", reader, template_reader);

                foreach(const parameter_reader_t::reader_map_t::value_type key_value, sensors_map)
                {
                    PRX_DEBUG_S("Reading Sensing Data: " << key_value.first);

                    const parameter_reader_t* data_template_reader = NULL;

                    if( key_value.second->has_attribute("template") )
                    {
                        template_name = key_value.second->get_attribute("template");
                        data_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);
                    }
                    sensor_t* new_data = parameters::create_from_loader<sensor_t > ("prx_simulation", key_value.second, "", data_template_reader, "");
                    parameters::initialize(new_data,key_value.second,"",data_template_reader,"");
                    sources_to_sensors[new_data->get_source()] = new_data;
                    all_sensors.push_back(new_data);
                    PRX_DEBUG_COLOR("Created new sensor Source:  " << new_data->get_source() , PRX_TEXT_GREEN);
                    if( data_template_reader != NULL )
                        delete data_template_reader;
                }
            }
            else
            {
                PRX_FATAL_S("Sensing initialized without any sensors!");
            }
            
        }
        
        void sensing_model_t::initialize_sensors(sim::simulator_t* sim)
        {
            PRX_DEBUG_COLOR("initializing sensors", PRX_TEXT_MAGENTA);
            if (state_space == NULL)
                state_space = sim->get_state_space();
            sensed_state = state_space->alloc_point();
            foreach(sensor_t* sensor, all_sensors)
            {
                PRX_DEBUG_COLOR("Sensor: " << sensor, PRX_TEXT_BLUE);
                sensor->initialize_sensor(sim);
            }
            PRX_ASSERT_MSG(state_space != NULL, "Must have a valid state space after initialization!");
        }


        void sensing_model_t::update_sensed_state ( const state_t* new_state)
        {
            PRX_ASSERT_MSG(state_space != NULL, "Cannont update sensed state with a NULL state space!");
            state_space->copy_point(sensed_state, new_state);
            
        }
        
        void sensing_model_t::sense(double time_step)
        {
            foreach(sensor_t* sensor, all_sensors)
            {
                if (sensor->fires(time_step))
                {
                    sensor->update_data();
                    sensor->reset_delay();
                }
            }
            
            foreach(sensing_info_t* info, all_sensing_info)
            {
                if (info->is_active())
                {
                    if (info->updates(time_step))
                    {
                        info->update_info();
                        info->reset_delay();
                    }
                }
            }
        }

        void sensing_model_t::set_sensing_info(const std::vector<sensing_info_t*>& sensing_info)
        {
           // PRX_DEBUG_COLOR("entering sensing _model::set_sensing_info : ",PRX_TEXT_LIGHTGRAY);
            all_sensing_info = sensing_info;
            foreach(sensing_info_t* info, all_sensing_info)
            {
                std::vector<sensor_t*> requested_sensors;

                foreach(std::string source, info->get_sensor_sources())
                {
                    if (sources_to_sensors.find(source) == sources_to_sensors.end())
                    {
                        PRX_DEBUG_COLOR("all_sensing_info::info::get_sensor_sources",PRX_TEXT_LIGHTGRAY);
                        PRX_FATAL_S ("Requested sensor: " << source << " does not exist in available sensor list.");
                    }

                    requested_sensors.push_back(sources_to_sensors[source]);
                }

                info->set_sensors(requested_sensors);
            }
        }
        
        pluginlib::ClassLoader<sensing_model_t>& sensing_model_t::get_loader()
        {
            return loader;
        }

    }
    
}
