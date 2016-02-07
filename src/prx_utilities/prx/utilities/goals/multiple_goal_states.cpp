/**
 * @file goal.cpp
 *  * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/goals/multiple_goal_states.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::util::multiple_goal_states_t, prx::util::goal_t)

namespace prx
{
    namespace util
    {

        multiple_goal_states_t::multiple_goal_states_t() { counter = 0; }

        multiple_goal_states_t::~multiple_goal_states_t() 
        { 
            for( int i=0; i<1000; i++ )
                space->free_point( goal_points[i] );
        }

        void multiple_goal_states_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            goal_t::init(reader, template_reader);

            if( parameters::has_attribute("goal_states", reader, template_reader) )
            {                    
            }
            else
            {
                PRX_INFO_S("No goal states will be added in the motion planner for this multiple goal states.");
            }

        }

        void multiple_goal_states_t::link_space(const space_t* inspace)
        {
            space = inspace;
            distance_metric->link_space(inspace);
            
            if( goal_points.size() == 0 )
            {
                for( int i=0; i<1000; i++ )
                    goal_points.push_back( inspace->alloc_point() );
            }
            counter = 0;
        }

        bool multiple_goal_states_t::satisfied(const space_point_t* state)
        {
            for( int i=0; i<counter; i++ )
                if( distance_metric->distance_function(goal_points[i], state) <= PRX_ZERO_CHECK )
                    return true;
            return false;
        }

        bool multiple_goal_states_t::satisfied(const space_point_t* state, double* distance)
        {
            for( int i=0; i<counter; i++ )
                *distance = distance_metric->distance_function(goal_points[i], state);
                if( *distance <= PRX_ZERO_CHECK )
                    return true;
            return false;
        }

        void multiple_goal_states_t::add_goal_state(const space_point_t* goal_state)
        {
            space->copy_point( goal_points[counter], goal_state);
            counter++;
            if( counter == 1000 )
                PRX_FATAL_S( "Run out of space in the multiple goal states " );
        }

        void multiple_goal_states_t::add_multiple_goal_states(const std::vector<space_point_t*>& goals)
        {
            foreach(space_point_t* point, goals)
            {
                space->copy_point( goal_points[counter], point);
                counter++;
                if( counter == 1000 )
                    PRX_FATAL_S( "Run out of space in the multiple goal states " );
            }
        }
        
        void multiple_goal_states_t::init_with_goal_state(const space_point_t* goal_state)
        {
            clear();
            add_goal_state(goal_state);
        }

        void multiple_goal_states_t::init_goal_states(const std::vector<space_point_t*>& goals)
        {
            clear();
            add_multiple_goal_states(goals);
        }
        
        void multiple_goal_states_t::clear()
        {
            counter = 0;
        }

    }
}
