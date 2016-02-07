/**
 * @file goal_state.cpp
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/goals/goal_state.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::util::goal_state_t, prx::util::goal_t)

namespace prx
{
    namespace util
    {

        goal_state_t::goal_state_t() { }

        goal_state_t::~goal_state_t()
        {
            distance_metric->clear();
            delete distance_metric;
        }

        void goal_state_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            goal_t::init(reader, template_reader);

            if( parameters::has_attribute("goal_state", reader, template_reader) )
            {
                state_vec = parameters::get_attribute_as< std::vector<double> >("goal_state", reader, template_reader);
            }
            else
            {
                PRX_WARN_S("Missing goal_state attribute in input files.");
            }
        }

        void goal_state_t::link_space(const space_t* inspace)
        {
            goal_t::link_space(inspace);
            point = space->alloc_point();
            if( state_vec.size() != 0 )
            {
                space->set_from_vector(state_vec, point);
                if( goal_points.size() != 0 )
                {
                    PRX_ASSERT(goal_points.size() <= 1);
                    space->copy_point(goal_points[0], point);
                }
                else
                {
                    goal_points.push_back(point);
                }
            }
        }       

        bool goal_state_t::satisfied(const space_point_t* state)
        {
            if( distance_metric->distance_function(point, state) <= PRX_ZERO_CHECK )
            {
                return true;
            }
            return false;
        }

        bool goal_state_t::satisfied(const space_point_t* state, double* distance)
        {
            *distance = distance_metric->distance_function(point, state);
            if( *distance <= PRX_ZERO_CHECK )
                return true;
            return false;
        }

        void goal_state_t::set_goal_state(const space_point_t* goal_state)
        {
            space->copy_point(point, goal_state);
            if( goal_points.size() != 0 )
            {
                PRX_ASSERT(goal_points.size() <= 1);
                space->copy_point(goal_points[0], point);
            }
            else
            {
                goal_points.push_back(point);
            }
        }
    }
}
