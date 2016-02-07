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

#include "planning/queries/manipulator_query.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::manipulator_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        using namespace manipulation;
        namespace rearrangement_manipulation
        {

            manipulator_query_t::manipulator_query_t()
            {
                state_space = NULL;
                control_space = NULL;
                mode = PRX_FULL_PATH;
                path_quality = PRX_FIST_PATH;
            }

            manipulator_query_t::~manipulator_query_t()
            {                
                clear();
            }

            void manipulator_query_t::clear()
            {
                manipulation_query_t::clear();
                
                for( unsigned i = 0; i < constraints.size(); ++i )
                {
                    constraints[i].clear();
                }
                constraints.clear();
                full_constraints.clear();
                solutions_costs.clear();
            }
            
            void manipulator_query_t::add_constraint(unsigned constraint, unsigned index)
            {
                constraints[index].insert(constraint);
            }

            void manipulator_query_t::add_constraints(std::set<unsigned> new_constraints, unsigned index)
            {
                constraints[index].insert(new_constraints.begin(),new_constraints.end());
            }
        }
    }
}
