/**
 * @file cost_function.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/cost_functions/cost_function.hpp"

#include <boost/function.hpp>
#include <pluginlib/class_loader.h>

namespace prx
{
    using namespace util;
    namespace sim
    {

        pluginlib::ClassLoader<cost_function_t> cost_function_t::loader("prx_simulation", "prx::sim::cost_function_t");

        cost_function_t::cost_function_t() { }

        cost_function_t::~cost_function_t() { }

        void cost_function_t::link_distance_function(distance_t in_dist)
        {
            dist = in_dist;
            this->cost_of_state = boost::bind(&cost_function_t::state_cost, this, _1);
            this->cost_of_trajectory = boost::bind(&cost_function_t::trajectory_cost, this, _1);
            this->heuristic = boost::bind(&cost_function_t::heuristic_cost, this, _1,_2);
        }

        pluginlib::ClassLoader<cost_function_t>& cost_function_t::get_loader()
        {
            return loader;
        }

    }
}
