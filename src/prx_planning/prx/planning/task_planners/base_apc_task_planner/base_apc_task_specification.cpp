/**
 * @file base_apc_task_specification.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/task_planners/base_apc_task_planner/base_apc_task_specification.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::plan::base_apc_task_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        base_apc_task_specification_t::base_apc_task_specification_t()
        {
            validity_checker = NULL;
            sampler = NULL;
            metric = NULL;
            local_planner = NULL;
        }

        base_apc_task_specification_t::~base_apc_task_specification_t()
        {
        }

        void base_apc_task_specification_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            specification_t::init(reader, template_reader);
        }
    }
}
