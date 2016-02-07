/**
 * @file default_uniform.cpp 
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

#include "prx/simulation/cost_functions/default_uniform.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::sim::default_uniform_t, prx::sim::cost_function_t)

namespace prx
{
	using namespace util;
    namespace sim
    {       

        double default_uniform_t::state_cost(const space_point_t* s)
        {
        	return 1;
        }

        double default_uniform_t::trajectory_cost(const trajectory_t& t)
        {
            PRX_ASSERT(t.size()>0);
            
            double cost = 0;
            trajectory_t::const_iterator i = t.begin();
            trajectory_t::const_iterator j = t.begin();
            j++;

            for ( ; j != t.end(); ++i,++j)
            {
                cost+=dist(*i,*j)*state_cost(*j);
            }
            return cost;
        }

        double default_uniform_t::heuristic_cost(const util::space_point_t* s,const util::space_point_t* t)
        {
            return dist(s,t);
        }

    }
}

// if(s->at(0) < 0)
// {
//     if(s->at(1) < 0)
//     {
//         return 50;
//     }
//     else
//     {
//         return 1;
//     }
// }
// else
// {
//     if(s->at(1) < 0)
//     {
//         return 20;
//     }
//     else
//     {
//         return 1;
//     }
// }