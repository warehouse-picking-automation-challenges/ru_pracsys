/**
 * @file distance_function.cpp 
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

#include "prx/utilities/distance_functions/distance_function.hpp"

#include <boost/function.hpp>
#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {

        pluginlib::ClassLoader<distance_function_t> distance_function_t::loader("prx_utilities", "prx::util::distance_function_t");

        distance_function_t::distance_function_t() { }

        distance_function_t::~distance_function_t() { }

        void distance_function_t::link_space(const space_t* space)
        {
            ref_space = space;
            this->dist = boost::bind(&distance_function_t::distance, this, _1, _2);
        }

        pluginlib::ClassLoader<distance_function_t>& distance_function_t::get_loader()
        {
            return loader;
        }

    }
}
