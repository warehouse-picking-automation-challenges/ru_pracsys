/**
 * @file obstacle_mapping.cpp 
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/mapping_functions/obstacle_mapping.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::util::obstacle_mapping_t, prx::util::mapping_function_t)

namespace prx
{
    namespace util
    {       

        obstacle_mapping_t::obstacle_mapping_t()
        {
            hide_mapping_t();
            mapping_name = "obstacle_mapping";
        }

        obstacle_mapping_t::~obstacle_mapping_t() { }

    }
}
