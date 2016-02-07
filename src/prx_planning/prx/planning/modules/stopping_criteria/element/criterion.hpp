/**
 * @file criterion.hpp 
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

#ifndef PRX_CRITERION_HPP
#define PRX_CRITERION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx
{
    namespace util 
    {
        class statistics_t;
        class goal_t;
    }
    
    namespace plan
    {
        class task_planner_t;
        class motion_planner_t;        
        

        /**
         * @anchor criterion_t
         *
         * The abstract criterion class is employed by planners of all sorts to determine
         * when they should stop execution, or perform certain operations.
         *
         * @brief <b> Abstract stopping criterion class. </b>
         *
         * @author Andrew Kimmel
         */
        class criterion_t
        {

            friend class task_planner_t;

          public:
            criterion_t();
            virtual ~criterion_t();

            /**
             * @brief Criterion initialization function.
             *
             * @param reader The \ref util::parameter_reader_t containing a dictionary of the criterion's parameters.
             * @param template_reader A \ref util::parameter_reader_t with default criterion parameters.
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL) = 0;

            /**
             * @brief Check this criterion for satisfaction.
             *
             * Checks to see if the criterion has been satisfied and returns a flag 
             * indicating such.
             *
             * @return A flag indicating if the criterion is satisfied.
             */
            virtual bool criterion_check() = 0;

            /**
             * Compute statistics on this criterion.
             * 
             * @brief Compute statistics on this criterion.
             *
             * @return The computed statistics for this individual criterion.
             */
            virtual util::statistics_t* compute_statistics();

            /**
             * Produce a string which contains statistics information.
             * 
             * @brief Produce a string which contains statistics information.
             *
             * @return The string containing statistical information.
             */
            virtual std::string print_statistics();

            /**
             * Link a motion planner to this criterion.
             * 
             * @brief Link a motion planner to this criterion.
             *
             * @param mp The motion planner to link to this criterion.
             */
            virtual void link_motion_planner(motion_planner_t* mp);

            /**
             * Link a goal to this criterion. Only certain criterion_t will actually
             * used this linked goal, thus the default implementation is empty.
             * 
             * @brief Link a goal to this criterion.
             *
             * @param new_goal The goal to link to this criterion.
             */
            virtual void link_goal(util::goal_t* new_goal);

            /**
             * Retrieve the criterion class loader.
             * 
             * @brief Retrieve the criterion class loader.
             *
             * @return The pluginlib class loader for criterion classes.
             */
            static pluginlib::ClassLoader<criterion_t>& get_loader();

            /**
             * Allows users to set new names (types) for the criterion, other than
             * the default ones. For example, the default type of a /ref goal_criterion_t
             * class is "goal", but could be changed to "radial_goal"
             * 
             * For stopping_criteria that have multiple of the same types of criterion,
             * it is useful to attach an identifier to the criterion.
             * 
             * @brief Set the type for this criterion.
             *
             * @param new_type The type we are setting this criterion to.
             */
            void set_type(const std::string& new_type);

            /**
             * Get a string indicating the type of this criterion.
             * 
             * @brief Get a string indicating the type of this criterion.
             *
             * @return The string stating what type this criterion is.
             */
            std::string get_type() const;

            /**
             * @brief Reset the criterion.
             *
             * Many criteria have interesting parameters, such as a count variable
             * or a timer.  This function resets those internal parameters to start
             * the criterion over in a sense.
             */
            virtual void reset() = 0; // TODO: Consider moving this to protected and rethinking who we should allow to reset criteria

            //        // We treat the satisfaction of a criterion as an "exception", so as to throw and catch it
            //        class criterion_satisfied : public std::runtime_error
            //        {
            //            public:
            //
            //                criterion_satisfied(std::string satisfaction_message) : std::runtime_error(satisfaction_message){ };
            //        };

          protected:
            /** @brief A pointer to the linked /ref motion_planner_t */
            motion_planner_t* linked_motion_planner;

            /** @brief A string identifier representing the type of this criterion class. */
            std::string criterion_type;

          private:
            /** @brief The pluginlib class loader for criterion_t classes. */
            static pluginlib::ClassLoader<criterion_t> loader;

        };

    }
}

#endif //PRX_CRITERION_HPP