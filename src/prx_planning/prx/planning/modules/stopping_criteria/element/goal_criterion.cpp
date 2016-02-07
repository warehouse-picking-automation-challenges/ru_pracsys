/**
 * @file iteration_criterion.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/state.hpp"
#include "prx/planning/modules/stopping_criteria/element/goal_criterion.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/goals/goal.hpp"


#include <pluginlib/class_list_macros.h>

namespace prx 
{ 
    using namespace util;
    using namespace sim;
    
    namespace plan 
    {

        PLUGINLIB_EXPORT_CLASS( prx::plan::goal_criterion_t, prx::plan::criterion_t)

        goal_criterion_t::goal_criterion_t()
        {
            PRX_DEBUG_S("Created a goal criterion");
            linked_goal = NULL;
        }

        goal_criterion_t::~goal_criterion_t()
        {
            
        }

        void goal_criterion_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader )
        {
            criterion_type = parameters::get_attribute_as<std::string>("criterion_type", reader, template_reader, "goal");
        }

        bool goal_criterion_t::criterion_check()
        {
        //    PRX_DEBUG_S ("Criterion Check: Number of states: " << linked_motion_planner->states_to_check.size());
            if (linked_goal == NULL)
            {
                PRX_ERROR_S ("Trying to check a goal criterion with no linked goal! Name: " << criterion_type);
                return false;
            }
            foreach(state_t* st, linked_motion_planner->states_to_check)
            {
                double distance;
                if (linked_goal->satisfied(st, &distance))
                    return true;
        //        PRX_ERROR_S ("Distance is: " << distance);
            }
            return false;
        }

        void goal_criterion_t::reset()
        {
            PRX_DEBUG_S ("Reseting goal criterion of type " << criterion_type);
        //    linked_goal = NULL;
        }

        void goal_criterion_t::link_goal(goal_t* new_goal)
        {
            PRX_DEBUG_S("Hey, the goal_criterion has a goal with type " << criterion_type);
            linked_goal = new_goal;
        }

    }
}