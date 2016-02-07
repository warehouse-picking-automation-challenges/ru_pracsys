/**
 * @file motion_planning_specification.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_MOTION_PLANNING_SPECIFICATION_HPP
#define	PRX_MOTION_PLANNING_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/problem_specifications/specification.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
        class goal_t;
    }
    
    namespace plan
    {

        /**
         * @anchor motion_planning_specification_t
         *
         * This class represents a problem instance to be solved by a motion planner.
         * It is a self-contained class which holds the start/goal pair as well as has
         * storage for answering trajectories and plans.
         *
         * @brief <b> General query for motion planners. </b>
         *
         * @author Athanasios Krontiris
         */
        class motion_planning_specification_t : public specification_t
        {

          public:
            motion_planning_specification_t();
            virtual ~motion_planning_specification_t();

            /**
             * @brief Initialize the planing query from input parameters.
             *
             * Initialize the planing query from input parameters.
             * 
             * @param reader The reader for the parameters.
             * @param template_reader A template reader for reading template parameters 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @brief Clear the plan and trajectory of this query.
             *
             * Clears the plan and the trajectory of the planning query in order to 
             * reuse the planning query with the same start and stopping criterion. 
             */
            virtual void clear();
            
            /** @copydoc specification_t::link_spaces(const util::space_t*, const util::space_t*) */
            virtual void link_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space);
            
            /**
             * @brief Clears the seeds
             * 
             * Clears the seeds
             */
            virtual void clear_seeds();
            
            /**
             * Set the seeds for the motion planner from a vector of doubles.
             * 
             * @brief Set the seeds for the motion planner from a vector of doubles.
             * 
             * @param s_vec The vector of vector of doubles to set the seeds to.
             */
            virtual void set_seeds_vec(const std::vector< std::vector< double > >& s_vec);
            
            /**
             * Set the seeds from a vector of state_t's. Will create new memory from them.
             * 
             * @brief Set the seeds from a vector of state_t's. Will create new memory from them.
             * 
             * @param in_seeds The vector of state_t's to add.
             */
            virtual void set_seeds(const std::vector<sim::state_t*>& in_seeds);
            
            /**
             * Add a single seed to the list of seed states.
             * 
             * @brief Add a single seed to the list of seed states.
             * 
             * @param state The state point to add to the list of seeds.
             */
            virtual void add_seed(const sim::state_t* state);
            
            /**
             * Get the list of seeds that were given to this specification.
             * 
             * @brief Get the list of seeds that were given to this specification.
             * 
             * @return The vector of seeds.
             */
            virtual std::vector<sim::state_t*>& get_seeds();
            
            /**
             * Get the validity of the seeds. Should come from motion planner after setting this specification.
             * 
             * @brief Get the validity of the seeds.
             * 
             * @return The vector of results.
             */
            virtual std::vector<bool>& get_validity();                        

          protected:
            /** @brief The states that we want our data structure to contain. */
            std::vector<sim::state_t*> seeds;
            /** @brief Temporary storage for reading seeds from input. */
            std::vector<std::vector<double> > seeds_vec;
            /** @brief Feedback from motion planner of the validity of seeds. */
            std::vector<bool> valid_seeds;
        };

    }
}

#endif

