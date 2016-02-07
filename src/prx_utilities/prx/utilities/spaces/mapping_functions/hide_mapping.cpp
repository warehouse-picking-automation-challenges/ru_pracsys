/**
 * @file hide_mapping.cpp 
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
#include "prx/utilities/spaces/mapping_functions/hide_mapping.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::util::hide_mapping_t, prx::util::mapping_function_t)

namespace prx
{
    namespace util
    {        

        void hide_mapping_t::embed() const { }

        void hide_mapping_t::invert() const { }


    }
}