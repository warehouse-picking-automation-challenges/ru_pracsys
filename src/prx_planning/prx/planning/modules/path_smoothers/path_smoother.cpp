/**
 * @file path_smoother.cpp 
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

#include "prx/planning/modules/path_smoothers/path_smoother.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

namespace prx 
{ 
    using namespace util;
    using namespace sim;
    
    namespace plan 
    {

//pluginlib::ClassLoader<path_smoother_t> path_smoother_t::loader("prx_planning", "prx::plan::path_smoother_t");

path_smoother_t::path_smoother_t() : world_model(NULL) 
{
}

path_smoother_t::~path_smoother_t() 
{
    if( metric != NULL )
        delete metric;
    if( sampler != NULL )
        delete sampler;
    if( local_planner != NULL )
        delete local_planner;
    if( validity_checker != NULL )
        delete validity_checker;
    if( stored_trajectory != NULL )
        delete stored_trajectory;
    if( stored_plan != NULL )
        delete stored_plan;
}

void path_smoother_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
{
    planner_t::init( reader, template_reader );
}

void path_smoother_t::link_world_model(world_model_t * const model)
{
    motion_planner_t::link_world_model( model );
    stored_plan = new plan_t( model->get_control_space() );
    stored_trajectory = new trajectory_t( model->get_state_space() );
}

void path_smoother_t::reset()
{
    input_query->plan = (*stored_plan);
    input_query->path = (*stored_trajectory);
}

void path_smoother_t::link_query(query_t* new_query)
{
    input_query = (motion_planning_query_t*)new_query;
    state_space = input_query->state_space;
    control_space = input_query->control_space;
    if( metric != NULL )
        metric->link_space(state_space);    
    (*stored_plan) = input_query->plan;
    (*stored_trajectory) = input_query->path;
}

void path_smoother_t::setup()
{
    //This function intentionally left empty
}

void path_smoother_t::resolve_query()
{
    PRX_WARN_S("Smoother: Cannot resolve the query by itself.");
}

bool path_smoother_t::succeeded() const
{
    return false;
}


    }
}

