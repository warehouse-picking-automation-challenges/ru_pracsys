/**
 * @file goal_state.hpp 
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
#pragma once

#ifndef PRX_GOAL_STATE_HPP
#define	PRX_GOAL_STATE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/goals/goal.hpp"

namespace prx
{
    namespace util
    {

        class parameter_reader_t;

        /**
         * A concrete implementation of a goal. Simple goal with a
         * state that we want to reach.
         * @brief  <b> Simple goal with a state that we want to reach.</b>
         * @author Athanasios Krontiris
         */
        class goal_state_t : public goal_t
        {

          public:
            goal_state_t();
            virtual ~goal_state_t();

            /** @copydoc goal_t::init(const parameter_reader_t*, const parameter_reader_t*) */
            virtual void init(const parameter_reader_t* reader, const parameter_reader_t* template_reader);

            /** @copydoc goal_t::link_space( const space_t* ) */
            virtual void link_space(const space_t* inspace);
            
            /** @copydoc goal_t::satisfied(const space_point_t* ) */
            virtual bool satisfied(const space_point_t* state);

            /** @copydoc goal_t::satisfied(const space_point_t* , double* ) */
            virtual bool satisfied(const space_point_t* state, double* distance);
            
            /**
             * Adds a new goal state to this goal_t object.
             * 
             * @brief Adds a new goal state. 
             * 
             * @param goal_state The new goal state that we want to reach.
             */
            virtual void set_goal_state(const space_point_t* goal_state);


          protected:
            /**
             * @brief Temporary storage for the goal state.
             */
            std::vector<double> state_vec;

            /**
             * @brief The space point representing the goal state.
             */
            space_point_t* point;
        };

    }
}

#endif	
