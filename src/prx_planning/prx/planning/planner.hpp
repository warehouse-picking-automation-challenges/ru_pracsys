/**
 * @file planner.hpp 
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

#ifndef PRX_PLANNER_HPP
#define PRX_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"

#include <pluginlib/class_loader.h>
#include <boost/any.hpp>

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
        class statistics_t;
        class distance_metric_t;
    }

    namespace sim
    {
        class trajectory_t;
    }

    namespace plan
    {

        class world_model_t;
        class sampler_t;
        class validity_checker_t;
        class local_planner_t;
        class query_t;
        class specification_t;

        /**
         * @anchor planner_t
         *
         * @ingroup planning
         *
         * Abstract planner class.  This class represents a general planner which 
         * receives a query and attempts to resolve it.  This class works with the
         * \ref world_model_t in order to create its plans.
         *
         * @brief <b> Abstract planner class for answering queries. </b>
         *
         * @author Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield
         */
        class planner_t
        {

          public:
            planner_t();

            virtual ~planner_t();

            /** 
             * @brief Initialize the planner from the given parameters.
             *
             * @param reader An input reader with a dictionary of parameters for the planner.
             * @param template_reader A template reader with a dictionary of default parameters.
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @brief Prepare the planner for execution.
             *
             * After all parameters have been loaded, setup prepares the planner for
             * actual execution. Setup happens after an init has been called but 
             * before planning has started.
             */
            virtual void setup() = 0;

            /**
             * @brief Associate a world model with the planner.
             *
             * This sets the context in which the planner will operate.
             * Motion and task planners will use the information provided
             * by the given world model to determine when goals have been
             * achieved.
             *
             * @param model The world model for the planner to use.
             */
            virtual void link_world_model(world_model_t * const model) = 0;

            /**
             * @brief Retrieves statistics regarding the planner's execution.
             *
             * @return An instance of a concrete planner's statistics class.
             */
            virtual const util::statistics_t* get_statistics() = 0;

            /**
             * @brief Link the problem specification for the planner.
             *
             * @param new_spec Link the problem specification for the planner.
             */
            virtual void link_specification(specification_t* new_spec) = 0;

            /**
             * @brief Link the query for the planner to solve.
             *
             * @param input_query The input query for the planner to solve.
             */
            virtual void link_query(query_t* input_query) = 0;

            /**
             * @brief Top-level function for executing the planner.
             *
             * This function sets the planner running, attempting to solve its given
             * query, and returns after its \ref stopping_criteria_t is satisfied, 
             * reporting whether or not its given \ref util::goal_t is achieved.
             *
             * @return A flag indicating success of achieving the planner's goal.
             */
            virtual bool execute() = 0;

            /** 
             * @brief Attempts to resolve the planner's query.
             * 
             * Resolves the planner's assigned query.  In general, you must first
             * link a query to the planner and run execute before attempting to
             * resolve a query.
             */
            virtual void resolve_query() = 0;

            /** 
             * @brief Reset the planner. This involves clearing all data structures.        
             */
            virtual void reset() = 0;

            /** 
             * @brief Successful query resolution check.
             *
             * Checks whether the given query has been succesfully resolved by
             * checking the embedded \ref util::goal_t.
             *
             * @return A flag indicating successful resolution of the planner query.
             */
            virtual bool succeeded() const = 0;

            /**
             * @brief Set the path name of the planner.
             *
             * Sets the entire pathname for the planner within the planning
             * heirarchy.  The pathname includes all parent planners and the actual
             * name of this planner at the end of the string.
             *
             * @param in_path The path name to set for this planner.
             *
             * @note The given pathname should accurately represent the planners location in the heirarchy.
             */
            void set_pathname(std::string in_path);

            /**
             * @brief Set an internal parameter of a planner at the given path.
             *
             * This function is responsible for setting internal parameters of a
             * planner which exists in the planning heirarchy.  The given path
             * parameter specifies the relative path of the planner to modify, and
             * also provides the name of the parameter to change, as well as the
             * value it should take.
             *
             * @param path The relative path of the planner we are setting the parameter for.
             * @param parameter_name The name of the parameter to set.
             * @param value The actual value for the parameter to take, given as a boost::any.
             */
            virtual void set_param(const std::string& path, const std::string& parameter_name, const boost::any& value) = 0;

            /**
             * @brief Update visualization information for this planner.
             *
             * Planners may need to visualize important information to the user,
             * such as goal locations, planning structures, and reasoning 
             * information.  This function updates the information to be visualized
             * based on the current planner parameters.
             */
            virtual void update_visualization() const;

            /**
             * @brief Retrieve the loader for this class instance.
             *
             * @return The pluginlib class loader for creating instances of planners.
             */
            static pluginlib::ClassLoader<planner_t>& get_loader();

          protected:

            /**
             * @brief Internal visualization information update
             *
             * This function is internally used to update visualization information,
             * which is access externally via update_visualization().
             */
            virtual void update_vis_info() const;

            /**
             * @brief Internal parameter setting function.
             *
             * When a path specifies that the planner to update is the current
             * planner, this function is called to actually handle the update of the
             * parameter.  It checks the given parameter name against known 
             * parameters for this planner, and appropriately casts the value to
             * update its internal information.
             *
             * @param parameter_name The name of the parameter to set.
             * @param value The actual value for the parameter to take, given as a boost::any.
             */
            virtual void set_param(const std::string& parameter_name, const boost::any& value);

            // Planning Modules
            /** @brief State validity checker. */
            validity_checker_t* validity_checker;

            /** @brief Random sampling module. */
            sampler_t* sampler;

            /** @brief Distance metric for nearest neighbors queries. */
            util::distance_metric_t* metric;

            /** @brief The planner's local planner for connecting states. */
            local_planner_t* local_planner;

            //planner name
            /** @brief The pathname of this planner. */
            std::string path;

            /** @brief A flag indicating whether the planner should visualize any informational geometries. */
            bool visualize;

          private:
            /** @brief The pluginlib loader which is returned by the get_loader() class. */
            static pluginlib::ClassLoader<planner_t> loader;

        };

    }
}

#endif
