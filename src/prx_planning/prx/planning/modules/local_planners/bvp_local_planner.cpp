/**
 * @file bvp_local_planner.cpp
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

#include "prx/planning/modules/local_planners/bvp_local_planner.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"

#include "prx/simulation/plan.hpp"

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::plan::bvp_local_planner_t, prx::plan::local_planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        bvp_local_planner_t::bvp_local_planner_t() { }

        bvp_local_planner_t::~bvp_local_planner_t() { }

        void bvp_local_planner_t::init(const parameter_reader_t * const reader, const parameter_reader_t * const template_reader)
        {
            local_planner_t::init(reader, template_reader);

            max_prop_length = parameters::get_attribute_as<double>("max_prop_length",reader,template_reader,PRX_INFINITY);
        }

        void bvp_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, trajectory_t& traj, bool connect)
        {
            world_model->steering_function(start, goal, plan);
            if( !connect && plan.length() > 0)
            {
                plan.trim(max_prop_length);
            }
            world_model->propagate_plan(start, plan, traj);
            // PRX_INFO_S("Result state: "<<world_model->get_state_space()->print_point(traj.back())<<" Actual state: "<<world_model->get_state_space()->print_point(goal));
        }

        void bvp_local_planner_t::steer(const state_t* start, const state_t* goal, plan_t& plan, state_t* result, bool connect)
        {
            world_model->steering_function(start, goal, plan);
            if( !connect && plan.length() > 0)
            {
                plan.trim(max_prop_length);
            }
            world_model->propagate_plan(start, plan, result);
        }

    }
}

