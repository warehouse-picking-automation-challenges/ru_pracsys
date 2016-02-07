/**
 * @file motion_planning_query.cpp
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

#include "prx/planning/queries/motion_planning_query.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"


#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS( prx::plan::motion_planning_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        motion_planning_query_t::motion_planning_query_t()
        {
            state_space = NULL;
            start_state = NULL;
            q_collision_type = PRX_NO_COLLISIONS;
            q_type = PRX_ADD_QUERY_POINTS_COLLISIONS;
            goal = NULL;
            solution_cost = 0;
        }

        motion_planning_query_t::~motion_planning_query_t()
        {
            if( state_space == NULL )
                PRX_FATAL_S("Planning query does not have a linked state space!");

            clear();
            start_vec.clear();
            if( start_state != NULL )
                state_space->free_point(start_state);
            if( goal != NULL )
            {
                delete goal;
            }
        }

        void motion_planning_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {            
            query_t::init(reader,template_reader);

            if( parameters::has_attribute("start_state", reader, template_reader) )
            {
                start_vec = parameters::get_attribute_as< std::vector<double> >("start_state", reader, template_reader);
            }
            else
            {
                PRX_WARN_S("Missing start state attribute from planning query.");
            }

            if( parameters::has_attribute("goal", reader, template_reader) )
            {
                std::string type = parameters::get_attribute_as<std::string > ("goal/type", reader, template_reader);
                goal = parameters::initialize_from_loader<goal_t>("prx_utilities",reader,"goal",template_reader,"goal");
            }
            else
            {
                PRX_WARN_S("Missing goal attribute from planning query.");
            }
            
            if( parameters::has_attribute("query_collision_type", reader, template_reader) )
            {
                std::string type = parameters::get_attribute("query_collision_type", reader, template_reader, "no_collisions");
                if( type == "no_collisions" )
                    q_collision_type = PRX_NO_COLLISIONS;
                else if( type == "lazy_collisions" )
                    q_collision_type = PRX_LAZY_COLLISIONS;
                else if( type == "active_collisions_reuse_edges" )
                    q_collision_type = PRX_ACTIVE_COLLISIONS_REUSE_EDGES;
                else if( type == "active_collisions_reporpagate_edges" )
                    q_collision_type = PRX_ACTIVE_COLLISIONS_REPROPAGATE_EDGES;
                
                else
                {
                    PRX_WARN_S("Wrong query collision type! No Collisions type will be used!");
                    q_collision_type = PRX_NO_COLLISIONS;
                }
            }
            else
            {
                PRX_WARN_S("Query collision type is not specified! No Collisions type will be used!");
                q_collision_type = PRX_NO_COLLISIONS;
            }
            
            if( parameters::has_attribute("query_type", reader, template_reader) )
            {
                std::string type = parameters::get_attribute("query_type", reader, template_reader, "add_query_points_collisions");
                if( type == "add_query_points_collisions" )
                    q_type = PRX_ADD_QUERY_POINTS_COLLISIONS;
                else if( type == "add_query_points_no_collisions" )
                    q_type = PRX_ADD_QUERY_POINTS_NO_COLLISIONS;
                else if( type == "near_query_points" )
                    q_type = PRX_NEAR_QUERY_POINTS;
                else
                {
                    PRX_WARN_S("Wrong query type! Add query points with collisions type will be used!");
                    q_type = PRX_ADD_QUERY_POINTS_COLLISIONS;
                }
            }
            else
            {
                PRX_WARN_S("Query type is not specified! Add query points with collisions type  will be used!");
                q_type = PRX_ADD_QUERY_POINTS_COLLISIONS;
            }
        }

        void motion_planning_query_t::clear()
        {
            if( state_space == NULL )
                PRX_FATAL_S("Planning query does not have a linked state space!");
            
            plan.clear();
            path.clear();
        }

        state_t* motion_planning_query_t::get_start_state() const
        {
            return start_state;
        }

        goal_t* motion_planning_query_t::get_goal() const
        {
            return goal;
        }

        void motion_planning_query_t::link_spaces(const space_t* state_space, const space_t* control_space)
        {
            this->state_space = state_space;
            start_state = this->state_space->alloc_point();
            if( !start_vec.empty() )
                this->state_space->copy_vector_to_point(start_vec, start_state);
            if(goal != NULL)
                goal->link_space(state_space);
            plan.link_control_space(control_space);
            path.link_space(state_space);
        }

        void motion_planning_query_t::set_start(state_t* start)
        {
            PRX_ASSERT(start != NULL);
            if( start_state != NULL )
            {
                //        PRX_INFO_S("Deleting the old start state in the planning query in order to replace it with a new");
//                state_space->free_point(start_state);
            }
            start_state = start;
        }
        
        void motion_planning_query_t::link_start(const sim::state_t* start)
        {
            if(start_state == NULL)
                start_state = state_space->clone_point(start);
            else
                state_space->copy_point(start_state,start);
        }

        void motion_planning_query_t::set_start_vec(const std::vector<double>& s_vec)
        {
            start_vec.clear();
            for( unsigned i = 0; i < s_vec.size(); i++ )
            {
                start_vec.push_back(s_vec[i]);
            }
        }

        void motion_planning_query_t::set_goal(goal_t* goal)
        {
            PRX_ASSERT(goal != NULL);
            if( this->goal != NULL )
            {
                //        PRX_INFO_S("Deleting the old goal state in the planning query in order to replace it with a new");
                delete this->goal;
            }
            this->goal = goal;
        }
    }
}
