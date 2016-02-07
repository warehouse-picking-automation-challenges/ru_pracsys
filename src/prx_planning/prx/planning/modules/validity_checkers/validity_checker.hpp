/**
 * @file validity_checker.hpp 
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

#ifndef PRX_VALIDITY_HPP
#define PRX_VALIDITY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/cost_functions/cost_function.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace plan
    {

        /**
         * @anchor validity_checker_t
         *
         * This class is one of the basic modules used by sampling-based motion planners.
         * We treat this as a black box that takes single states or entire trajectories
         * and reports whether or not they are valid, given the individual validity
         * checker's method of state validation.
         *
         * @brief <b> Abstract validity checker class for verifying states. </b>
         *
         * @author Zakary Littlefield
         */
        class validity_checker_t
        {

          public:

            validity_checker_t();

            virtual ~validity_checker_t(){ }

            /** 
             * Initialize the validity checker from the given parameters.
             * 
             * @brief Initialize the validity checker from the given parameters.
             *
             * @param reader An input reader with a dictionary of parameters for the validity checker.
             * @param template_reader A template reader with a dictionary of default parameters.
             */          
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
            
//            /**
//             * @brief Validity checker initialization method.
//             *
//             * @param reader A \ref util::parameter_reader_t containing a dictionary of validity checker parameters.
//             */
//            virtual void init(const util::parameter_reader_t* reader){ }

            /**
             * @brief Validity check for a single state point.
             *
             * @param point The specific state to be checked for validity.
             */
            virtual bool is_valid(const sim::state_t* point) = 0;

            /**
             * @brief Validity check for an entire trajectory.
             *
             * @param input The input trajectory to be checked for validity.
             */
            virtual bool is_valid(const sim::trajectory_t& input);

            /**
             * @brief Validity check for an entire trajectory.
             *
             * @param input The input trajectory to be checked for validity.
             */
            virtual bool is_valid(const sim::trajectory_t& input,unsigned min, unsigned max);

            /**
             * @brief Truncate a trajectory to make it valid.
             *
             * This method takes an input trajectory and truncates it so that all the 
             * valid states up to the first invalid state is retained.
             *
             * @param space A (potentially?) unused space parameter.
             * @param input The input trajectory to truncate.
             * @param result The resulting trajectory after truncation.
             */
            virtual void valid_trajectory(const util::space_t* space, const sim::trajectory_t& input, sim::trajectory_t& result); //TODO : Remove space parameter as it is unused in the method?

            /**
             * @brief Truncate a trajectory to make it valid.
             *
             * This method takes an input trajectory and truncates it so that all the 
             * valid states up to the first invalid state is retained.

             * @param input The input trajectory to truncate.
             */
            virtual void valid_trajectory(sim::trajectory_t& input);

            /**
             * @brief Truncate a trajectory up to a maximum size to make it valid.
             *
             * This method takes an input trajectory and truncates it so that all the 
             * valid states up to the first invalid state, or up to a maximum size is 
             * retained.
             *
             * @param space A (potentially?) unused space parameter.
             * @param input The input trajectory to truncate.
             * @param result The resulting trajectory after truncation.
             * @param size The maximum allowed size of the resulting trajectory.
             */
            virtual void valid_trajectory_prefix(const util::space_t* space, const sim::trajectory_t& input, sim::trajectory_t& result, unsigned int size); //TODO : Remove space parameter as it is unused in the method?

            /**
             * @brief Link a world model for this validity checker to use.
             *
             * @param model The world model to link to this validity checker.
             */
            virtual void link_model(world_model_t* model);

            /**
             * @brief Link a distance function for this validity checker to use.
             *
             * @param model The distance function to link to this validity checker.
             */
            virtual void link_distance_function(util::distance_t dist);

            /**
             * @brief Retrieve the validity checker class loader.
             *
             * @return The validity checker pluginlib class loader object.
             */
            static pluginlib::ClassLoader<validity_checker_t>& get_loader();

            /** @brief The function for determining a state's cost */
            sim::state_cost_t state_cost;

            /** @brief The function for determining a trajectories's cost */
            sim::trajectory_cost_t trajectory_cost;

            /** @brief The function for determining a trajectories's cost */
            sim::heuristic_cost_t heuristic;

          protected:

            std::string cost_function_name;

            sim::cost_function_t* cost_function;

            /** @brief The linked world model for this validity checker. */
            world_model_t* world_model;

          private:
            /** @brief The pluginlib class loader for validity checkers. */
            static pluginlib::ClassLoader<validity_checker_t> loader;

        };

    }
}

#endif
