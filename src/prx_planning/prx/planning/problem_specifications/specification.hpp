/**
 * @file specification.hpp
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

#ifndef PRX_SPECIFICATION_HPP
#define	PRX_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/state.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
        class distance_metric_t;
    }

    namespace plan
    {
        class world_model_t;
        class sampler_t;
        class validity_checker_t;
        class local_planner_t;
        class stopping_criteria_t;
        class astar_module_t;

        /**
         * @anchor specification_t
         *
         * The abstract specification class represents some specification that needs to be answered.
         * Queries are used by motion planners to define start/goal pairs, and by task
         * planners for high-level objectives.
         *
         * @brief <b> Abstract specification class. </b>
         * 
         * @Author Athanasios Krontiris
         */
        class specification_t
        {

          public:

            specification_t();

            virtual ~specification_t();

            /**
             * @brief Initialize the specification from input parameters.
             *
             * @param reader The reader for the parameters.
             * @param template_reader A template reader for reading template parameters 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @brief Clear the plan and trajectory of this query.
             *
             * Clears the plan and the trajectory of the planning query in order to 
             * reuse the planning query with the same start and stopping criterion. 
             */
            virtual void clear();
            
            /**
             * Prepare the planning specification to be linked to the children
             * 
             * @brief Prepare the planning specification to be linked to the children
             */
            virtual void setup(world_model_t * const model);

            /**
             * @brief Returns the stopping criterion for the query.
             *
             * Returns the stopping criterion for the query.
             * 
             * @return The stopping criterion. 
             */
            virtual stopping_criteria_t* get_stopping_criterion() const;

            /**
             * @brief Links the state and control spaces that the planning query is working over.
             *
             * Links the state and control spaces that the planning query is working over.
             * 
             * @param state_space The state space.
             * @param control_space The control space.
             */
            virtual void link_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space);

            /**
             * @brief Set the stopping criterion for the query.
             *
             * The motion_planning_specification_t will own this stopping criterion.Planning_query_t will
             * delete this stopping criterion at the end of the execution.
             *
             * @param criterion The stopping criterion that we want to set to the planning query.
             */
            virtual void set_stopping_criterion(stopping_criteria_t* criterion);

            /**
             * Set the flag for gathering the statistics.
             * 
             * @brief Set the flag for gathering the statistics.
             * 
             * @param flag True gather statistics, False do not gather the statistics.
             */
            virtual void gather_statistics(bool flag);


            /** @brief State space over which this query is operating. */
            const util::space_t* state_space;
            /** @brief Control space over which this query is operating. */
            const util::space_t* control_space;


            /**
             * @brief Retrieve the pluginlib class loader for queries.
             *
             * @return The pluginlib class loader for queries.
             */
            static pluginlib::ClassLoader<specification_t>& get_loader();

            /** @brief The name of the planner this specification is supplied to. */
            std::string planner_name;

            // Planning Modules
            /** @brief State validity checker. */
            validity_checker_t* validity_checker;

            /** @brief Random sampling module. */
            sampler_t* sampler;

            /** @brief Distance metric for nearest neighbors queries. */
            util::distance_metric_t* metric;

            /** @brief The planner's local planner for connecting states. */
            local_planner_t* local_planner;
            
            /** @brief The A8 module for heuristic search for the planners that they need that. */
            astar_module_t* astar;

          protected:
            /** @brief The stopping criteria for answering this query. */
            stopping_criteria_t* stopping_criteria;
            /** @brief Whether the motion planner should perform data gathering or not.*/
            bool with_statistics;

          private:
            /** @brief The specification pluginlib class loader object. */
            static pluginlib::ClassLoader<specification_t> loader;

        };

    }
}

#endif
