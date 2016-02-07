/**
 * @file config_sensor.cpp 
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

#include "prx/simulation/sensing/sensors/config_sensor.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/system_graph.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS(prx::sim::config_sensor_t, prx::sim::sensor_t);

namespace prx
{
    using namespace util;
    
    namespace sim
    {
        
        config_sensor_t::config_sensor_t()
        {
            update_obstacle_configs = true;
        }

        config_sensor_t::~config_sensor_t()
        {
        }
        
        void config_sensor_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            sensor_t::init(reader, template_reader);
        
        }
        
        void config_sensor_t::initialize_sensor(simulator_t* sim)
        {
            system_graph_t sys_graph;
            sim->update_system_graph(sys_graph);
            sys_graph.get_plants(sensed_plants);
            foreach(system_ptr_t ob, sim->get_obstacles() | boost::adaptors::map_values)
            {
              //  PRX_DEBUG_COLOR("The number of obstacles in the environment: "<<++i,PRX_TEXT_LIGHTGRAY);
                sensed_obstacles.push_back(dynamic_cast<obstacle_t*>(ob.get()));
            }
        }
        
        void config_sensor_t::update_data()
        {
//            PRX_DEBUG_S ("UPDATE CONFIGS!");
            unsigned index = 0;
            foreach(plant_t* plant, sensed_plants)
            {
                plant->update_phys_configs(plant_configs, index);
            }
//            foreach(config_list_element_t elem, plant_configs)
//            {
//                PRX_DEBUG_COLOR("Name: " << elem.first << " conf: " << elem.second, PRX_TEXT_CYAN);
//            }
            
            if (update_obstacle_configs)
            {
                index = 0;
                foreach(obstacle_t* obst, sensed_obstacles)
                {
                    obst->update_phys_configs(obstacle_configs, index);
                }
                update_obstacle_configs = false;
            }
//            foreach(config_list_element_t elem, obstacle_configs)
//            {
//                PRX_DEBUG_COLOR("Name: " << elem.first << " conf: " << elem.second, PRX_TEXT_MAGENTA);
//            }
        }
        
        void config_sensor_t::set_update_obstacles(bool update_flag)
        {
            update_obstacle_configs = update_flag;
        }
        
        util::config_list_t config_sensor_t::get_configs()
        {
            config_list_t returned_list;
            returned_list.insert(returned_list.begin(),plant_configs.begin(), plant_configs.end());
//            foreach(config_list_element_t elem, returned_list)
//            {
//                PRX_DEBUG_COLOR("Name: " << elem.first << " conf: " << elem.second, PRX_TEXT_CYAN);
//            }
            returned_list.insert(returned_list.end(), obstacle_configs.begin(), obstacle_configs.end());
            return returned_list;
        }

        util::config_list_t config_sensor_t::get_plant_configs()
        {
            //PRX_DEBUG_COLOR("plant configs"<< plant_configs, PRX_TEXT_LIGHTGRAY);
            return plant_configs;
        }
        
        util::config_list_t config_sensor_t::get_obstacle_configs()
        {
            return obstacle_configs;
        }

        
    }
}

