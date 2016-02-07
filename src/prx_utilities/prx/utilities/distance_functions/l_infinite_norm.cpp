/**
 * @file l_infinite_norm.cpp 
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
#include "prx/utilities/distance_functions/l_infinite_norm.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::util::l_infinite_norm_t, prx::util::distance_function_t)

namespace prx
{
    namespace util
    {       
        double l_infinite_norm_t::distance(const space_point_t* p1, const space_point_t* p2)
        {
            double* s1 = const_cast<double*>(&p1->memory[0]);
            double* s2 = const_cast<double*>(&p2->memory[0]);
            
            // ORIGINAL L_\infty norm
            unsigned dim = p1->memory.size();
            double max_value = 0;
            double val = 0;
            for(unsigned i=0;i<dim;i++)
            {
                val = fabs(s1[i] - s2[i]);
                if(val > max_value)
                    max_value = val;
            }
            return max_value;
            
            // TEST: Manhattan distance
            // double total_dist = 0;
            // unsigned dim = p1->memory.size();
            // for( unsigned i=0; i<dim; ++i )
            // {
            //     total_dist += fabs(s1[i] - s2[i]);
            // }
            // return total_dist;            
        }
    }
}
