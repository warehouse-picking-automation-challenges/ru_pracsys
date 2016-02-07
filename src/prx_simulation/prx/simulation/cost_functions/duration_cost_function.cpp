/**
 * @file duration_cost_function.cpp 
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

#include "prx/simulation/cost_functions/duration_cost_function.hpp"
#include "prx/simulation/systems/system.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::sim::duration_cost_function_t, prx::sim::cost_function_t)

namespace prx
{
	using namespace util;
    namespace sim
    {       
        duration_cost_function_t::duration_cost_function_t()
        {
        }
        duration_cost_function_t::~duration_cost_function_t()
        {
        }

        double duration_cost_function_t::state_cost(const space_point_t* s)
        {
            return 1;
        }

        double duration_cost_function_t::trajectory_cost(const trajectory_t& t)
        {
            return t.size();
        }

        double duration_cost_function_t::heuristic_cost(const util::space_point_t* s,const util::space_point_t* t)
        {
            return (dist(s,t)/4)/simulation::simulation_step;
        }


    }
}
