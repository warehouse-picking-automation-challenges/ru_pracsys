/**
 * @file manipulation_query.cpp
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

#include "planning/queries/manipulator_mp_query.hpp"
#include "planning/problem_specifications/manipulator_specification.hpp"
#include "prx/planning/modules/heuristic_search/astar_module.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::manipulator_mp_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            manipulator_mp_query_t::manipulator_mp_query_t() 
            {
                search_mode = astar_module_t::PRX_NO_EDGE_INFO;
            }

            manipulator_mp_query_t::~manipulator_mp_query_t()
            {
                constraints.clear();
            }

            void manipulator_mp_query_t::clear()
            {
                motion_planning_query_t::clear();
                constraints.clear();
                full_constraints.clear();
            }

            void manipulator_mp_query_t::add_constraint(unsigned constraint)
            {
                constraints.insert(constraint);
            }

            void manipulator_mp_query_t::add_constraints(std::set<unsigned> new_constraints)
            {
                constraints.insert(new_constraints.begin(), new_constraints.end());
            }
            
            void manipulator_mp_query_t::set_extra_constraint(unsigned constraint)
            {
                has_extra_constraint = true;
                extra_constraint = constraint;
            }
        }
    }
}
