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
#include "prx/utilities/goals/bounded_goal_region.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::util::bounded_goal_region_t, prx::util::goal_t)

namespace prx
{
    namespace util
    {       

        bounded_goal_region_t::bounded_goal_region_t() { }

        bounded_goal_region_t::~bounded_goal_region_t() { }

        void bounded_goal_region_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            goal_t::init(reader, template_reader);
            const std::vector<double> min = parameters::get_attribute_as<std::vector<double> >("min", reader, template_reader);
            const std::vector<double> max = parameters::get_attribute_as<std::vector<double> >("max", reader, template_reader);

            unsigned dimension = min.size();

            for( unsigned i = 0; i < dimension; i++ )
            {
                bounds.push_back(new bounds_t());
            }
            bounds::set_bounds(bounds, min, max);
        }

        void bounded_goal_region_t::link_space(const space_t* inspace)
        {
            PRX_DEBUG_S("Bounded goal region link space");
            goal_t::link_space(inspace);

            std::vector<double> new_point;
            new_point.resize(bounds.size());

            //check the bounds and update them accordingly
            const std::vector<bounds_t*> space_bounds = space->get_bounds();

            for( unsigned i = 0; i < bounds.size(); i++ )
            {
                bounds[i]->intersect(*space_bounds[i]);
            }
            for( unsigned i = 0; i < bounds.size(); i++ )
            {
                double low, high;

                bounds[i]->get_bounds(low, high);
                new_point[i] = (low + high) / 2;
            }

            point = space->alloc_point();
            space->set_from_vector(new_point, point);
            goal_points.push_back(point);

            // for( unsigned i = 0; i < bounds.size(); i++ )
            // {
            //     PRX_INFO_S(*bounds[i]);
            // }
        }

        bool bounded_goal_region_t::satisfied(const space_point_t* state)
        {
            unsigned dimension = bounds.size();
            for( unsigned int i = 0; i < dimension; ++i )
            {
                if( state->memory[i] > bounds[i]->get_upper_bound() || state->memory[i] < bounds[i]->get_lower_bound() )
                    return false;
            }
            return true;
        }

        bool bounded_goal_region_t::satisfied(const space_point_t* state, double* distance)
        {
            *distance = distance_metric->distance_function(point, state);
            return this->satisfied(state);
        }

    }
}
