/**
 * @file distance_metric.cpp 
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

#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx 
 { 
 namespace util 
 {

pluginlib::ClassLoader<distance_metric_t> distance_metric_t::loader("prx_utilities", "prx::util::distance_metric_t");

distance_metric_t::distance_metric_t( )
{
    nr_points = 0;
    function_name = "default_euclidean";
    space = NULL;
}

void distance_metric_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
{
    function_name = parameters::get_attribute_as<std::string>("distance_function",reader,template_reader,"default_euclidean");
    PRX_INFO_S("Distance_function: "<<function_name);
}

unsigned distance_metric_t::get_nr_points( )
{
    return nr_points;
}

void distance_metric_t::link_space( const space_t* inspace )
{
    if (space != inspace)
    {
        space = inspace;
        unsigned dim = space->get_dimension();
        gamma_val = exp(1)*(1+1.0/dim)+.001;        
//        nr_points = 0;
        this->clear();
        space_measure = 1;
        const std::vector<bounds_t*>& sp_bounds = space->get_bounds();
        for( unsigned i=0; i<space->get_dimension(); ++i )
            space_measure *= sp_bounds[i]->get_upper_bound() - sp_bounds[i]->get_lower_bound();

        if(function_name=="default")
        {
            function_name = "default_euclidean";
        }
        function = distance_function_t::get_loader().createUnmanagedInstance("prx_utilities/"+function_name);
        function->link_space(inspace);
        distance_function = function->dist;
    }
    
}

pluginlib::ClassLoader<distance_metric_t>& distance_metric_t::get_loader()
{
    return loader;
}

} 
 }
