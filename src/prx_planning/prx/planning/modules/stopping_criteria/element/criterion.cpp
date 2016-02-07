/**
 * @file criterion.cpp 
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

#include "prx/planning/modules/stopping_criteria/element/criterion.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/utilities/statistics/statistics.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    using namespace util;
    namespace plan
    {

        pluginlib::ClassLoader<criterion_t> criterion_t::loader("prx_planning", "prx::plan::criterion_t");

        criterion_t::criterion_t() { }

        criterion_t::~criterion_t() { }

        void criterion_t::link_motion_planner(motion_planner_t* mp)
        {
            linked_motion_planner = mp;
            //    this->reset();
        }

        void criterion_t::link_goal(goal_t* new_goal) {
            //    this->link_goal(new_goal);
            //    PRX_WARN_S ("Link goal not implemented in criterion: " << criterion_type);
        }

        statistics_t* criterion_t::compute_statistics()
        {
            //    PRX_WARN_S ("No statistics implemented for stopping criterion!");
            return NULL;
        }

        std::string criterion_t::print_statistics()
        {
            //    PRX_WARN_S ("No statistics implemented for stopping criterion!");
            return std::string("");
        }

        void criterion_t::set_type(const std::string& new_type)
        {
            criterion_type = new_type;
        }

        std::string criterion_t::get_type() const
        {
            return criterion_type;
        }

        pluginlib::ClassLoader<criterion_t>& criterion_t::get_loader()
        {
            return loader;
        }

    }
}