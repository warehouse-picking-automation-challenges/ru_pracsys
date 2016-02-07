/**
 * @file default_euclidean.hpp 
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
#include "prx/utilities/distance_functions/default_euclidean.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::util::default_euclidean_t, prx::util::distance_function_t)

namespace prx
{
    namespace util
    {       

        double default_euclidean_t::distance(const space_point_t* s1, const space_point_t* s2)
        {
            return this->ref_space->distance(s1, s2);
        }

    }
}
