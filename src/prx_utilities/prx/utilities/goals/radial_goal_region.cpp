/**
 * @file radial_goal_region.cpp
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
#include "prx/utilities/goals/radial_goal_region.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::util::radial_goal_region_t, prx::util::goal_t)

namespace prx
{
    namespace util
    {        

        radial_goal_region_t::radial_goal_region_t() { }

        radial_goal_region_t::~radial_goal_region_t() { }

        void radial_goal_region_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            PRX_DEBUG_COLOR("Inside radial goal region init!", PRX_TEXT_BLUE);
            goal_t::init(reader, template_reader);

            if( parameters::has_attribute("goal_state", reader, template_reader) )
            {
                PRX_DEBUG_S("Reading in goal state");
                state_vec = parameters::get_attribute_as< std::vector<double> >("goal_state", reader, template_reader);
                for( unsigned i = 0; i < state_vec.size(); i++ )
                {
                    PRX_DEBUG_COLOR("Goal: " << state_vec[i], PRX_TEXT_RED);
                }
            }
            else
            {
                PRX_WARN_S("Missing goal_state attribute in input files.");
            }
            if( parameters::has_attribute("radius", reader, template_reader) )
            {
                PRX_DEBUG_S("Reading in goal radius");
                radius = parameters::get_attribute_as< double >("radius", reader, template_reader);
                PRX_DEBUG_COLOR("Radius is: " << radius, PRX_TEXT_CYAN);
            }
            else
            {
                PRX_FATAL_S("Missing radius attribute in input files for the radial_goal_region.");
            }
        }

        void radial_goal_region_t::link_space(const space_t* inspace)
        {
            PRX_DEBUG_S("Radial goal region link space");
            goal_t::link_space(inspace);
            point = space->alloc_point();
            space->set_from_vector(state_vec, point);
            goal_points.push_back(point);
        }

        bool radial_goal_region_t::satisfied(const space_point_t* state)
        {
            if( distance_metric->distance_function(point, state) <= radius )
                return true;
            return false;
        }

        bool radial_goal_region_t::satisfied(const space_point_t* state, double* distance)
        {
            *distance = distance_metric->distance_function(point, state);
            if( *distance <= radius )
                return true;
            return false;
        }

        double radial_goal_region_t::get_radius() const
        {
            return radius;
        }

        const std::vector<double>& radial_goal_region_t::get_goal_vec() const
        {
            return state_vec;
        }

        void radial_goal_region_t::set_goal(distance_metric_t* d_metric, const std::vector<double>& s_vec, double r)
        {
            distance_metric = d_metric;
            state_vec = s_vec;
            radius = r;
        }

    }
}
