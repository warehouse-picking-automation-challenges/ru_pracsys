/**
 * @file local_planner.cpp 
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


#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"


namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace plan
    {

        pluginlib::ClassLoader<local_planner_t> local_planner_t::loader("prx_planning", "prx::plan::local_planner_t");

        void local_planner_t::init(const parameter_reader_t * const reader, const parameter_reader_t * const template_reader)
        {
            duration_step = parameters::get_attribute_as<double>("duration_step", reader, template_reader, 0);
        }

        void local_planner_t::propagate(const state_t* start, const plan_t& plan, trajectory_t& traj)
        {
            world_model->propagate_plan(start, plan, traj);
        }

        void local_planner_t::propagate_step(const state_t* start, const plan_t& plan, state_t* state)
        {
            world_model->propagate_plan(start, plan, state);
        }

        void local_planner_t::set_duration_step(double step)
        {
            duration_step = step;
        }

        void local_planner_t::link_model(world_model_t* model)
        {
            world_model = model;
        }

        void local_planner_t::link_sampler(sampler_t* in_sampler)
        {
            sampler = in_sampler;
        }

        void local_planner_t::link_metric(distance_metric_t* dist_metric)
        {
            metric = dist_metric;
        }

        void local_planner_t::link_state_space(const space_t* space)
        {
            state_space = space;
        }

        void local_planner_t::link_control_space(const space_t* space)
        {
            control_space = space;
        }

        pluginlib::ClassLoader<local_planner_t>& local_planner_t::get_loader()
        {
            return loader;
        }

    }
}
