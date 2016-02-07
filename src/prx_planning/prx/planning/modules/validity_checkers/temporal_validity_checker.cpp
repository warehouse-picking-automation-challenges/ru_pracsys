/**
 * @file temporal_validity_checker.cpp 
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


#include "prx/planning/modules/validity_checkers/temporal_validity_checker.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::temporal_validity_checker_t, prx::plan::validity_checker_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        bool temporal_validity_checker_t::is_valid(const state_t* point)
        {
            return world_model->valid_state(point);
        }

        bool temporal_validity_checker_t::is_valid(const trajectory_t& input)
        {
            return !input.in_collision();
        }

    }
}