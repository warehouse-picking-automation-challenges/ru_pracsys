/**
 * @file geometry_sensor.cpp 
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


#include "prx/simulation/sensing/sensors/geometry_sensor.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"


#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS(prx::sim::geometry_sensor_t, prx::sim::sensor_t);

namespace prx
{
    using namespace util;
    
    namespace sim
    {
        
        geometry_sensor_t::geometry_sensor_t()
        {
        }

        geometry_sensor_t::~geometry_sensor_t()
        {
        }
        
        void geometry_sensor_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            sensor_t::init(reader, template_reader);
            
        }
        
        void geometry_sensor_t::initialize_sensor(simulator_t* sim)
        {
            sim->get_sensed_geoms(sensed_geometries);
            foreach(system_ptr_t sys, sim->get_obstacles() | boost::adaptors::map_values)
            {
                sys->get_sensed_geoms(sensed_geometries);
            }
        }
        
        void geometry_sensor_t::update_data()
        {
//            PRX_DEBUG_S ("UPDATE GEOMETRY!");
        }
        
        
        const util::geom_map_t& geometry_sensor_t::get_sensed_geometries()
        {
            return sensed_geometries;
        }
        
    }
}