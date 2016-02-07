/**
 * @file base_apc_task_query.cpp
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

#include "prx/planning/task_planners/base_apc_task_planner/base_apc_task_query.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"


#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS( prx::plan::base_apc_task_query_t, prx::plan::query_t)


namespace prx
{
    using namespace util;
    namespace plan
    {

        void base_apc_task_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            motion_planning_query_t::init(reader,template_reader);
        }

    }
}