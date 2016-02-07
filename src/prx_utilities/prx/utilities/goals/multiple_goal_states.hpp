/**
 * @file multiple_goal_states.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_MULTIPLE_GOAL_STATES_HPP
#define	PRX_MULTIPLE_GOAL_STATES_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/goals/goal.hpp"

namespace prx
{
    namespace util
    {

        class parameter_reader_t;

        /** * 
         * A concrete implementation of a goal_T. Multiple state goals that we want to reach.
         * @brief  <b> Multiple state goals.</b>
         * 
         * @author Athanasios Krontiris
         */
        class multiple_goal_states_t : public goal_t
        {

          public:
            multiple_goal_states_t();
            virtual ~multiple_goal_states_t();

            /** @copydoc goal_t::init(const parameter_reader_t*, const parameter_reader_t*) */
            virtual void init(const parameter_reader_t* reader, const parameter_reader_t* template_reader = NULL);

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
            virtual void add_goal_state(const space_point_t* goal_state);
            
            /**
             * Adds many goal states.
             * 
             * @brief Adds many goal states.
             * 
             * @param goals The new goals that we want to add.
             */
            virtual void add_multiple_goal_states(const std::vector<space_point_t*>& goals);
            
            /**
             * Adds a new goal state after cleans all the previous states.
             * 
             * @brief Adds a new goal state after cleans all the previous states.
             * 
             * @param goal_state The new goal state that we want to initialize our goal_t with.
             * 
             */
            virtual void init_with_goal_state(const space_point_t* goal_state);
            
            /**
             * Adds many goal states after it clears the previous goal states.
             * 
             * $brief Adds many goal states after it clears the previous goal states.
             * 
             * @param goals The new goals that we want to add.
             */
            virtual void init_goal_states(const std::vector<space_point_t*>& goals);

            /**
             * Get the pluginlib loader for this base class.
             * @brief Get the pluginlib loader for this base class.
             * @return The pluginlib loader.
             */
            static pluginlib::ClassLoader<multiple_goal_states_t>& get_loader();

            
            virtual int get_number_of_points() 
            {
                return counter;
            }

            virtual void clear();

          protected:
            /**
             * @brief Temporary storage for the goal states from the input.
             */
            std::vector<std::vector<double> > states_vec;

            int counter;

        };

    }
}

#endif	
