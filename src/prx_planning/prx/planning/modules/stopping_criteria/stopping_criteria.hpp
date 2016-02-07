/**
 * @file stopping_criteria.hpp 
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

#ifndef PRX_STOPPING_CRITERIA_HPP
#define PRX_STOPPING_CRITERIA_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/modules/stopping_criteria/element/criterion.hpp"

namespace prx
{
    namespace util
    {
        class statistics_t;
        class goal_t;
    }

    namespace plan
    {

        class motion_planner_t;

        /**
         * @anchor stopping_criteria_t
         *
         * The stopping criteria class represents a collection of individual \ref
         * criterion_t classes and represents an entire logical set of criteria.
         *
         * @brief <b> Collection of individual \ref criterion_t objects. </b>
         *
         * @author Andrew Kimmel
         */
        class stopping_criteria_t
        {

          public:

            stopping_criteria_t();
            virtual ~stopping_criteria_t();

            /**
             * Stopping criteria initialization function.
             * 
             * @note The stopping criteria class can be initialized using template readers,
             * which must be passed to the stopping criteria with the correct path set
             * in them.
             * 
             * @brief Stopping criteria initialization function.
             *
             * @param reader The \ref util::parameter_reader_t from which to initialize this stopping criteria.
             * @param template_reader A \ref util::parameter_reader_t with template parameters for the stopping criteria.
             */
            void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @brief Satisfaction testing for the stopping criteria.
             *
             * Checks all encapsulated \ref criterion_t classes for satisfaction.  This
             * function returns true if all individual criteria have been satisfied if
             * the check type is set to ALL or returns true if a single criterion is
             * satisfied if the check type is set to SINGLE.
             *
             * @return A flag indicating whether the criteria are satisfied.
             */
            bool satisfied();

            /**
             * Reset the criteria so as to start it fresh.
             * 
             * @brief Reset the criteria so as to start it fresh.
             */
            void reset();

            /**
             * Link a motion planner to this set of criteria.
             * 
             * @brief Link a motion planner to this set of criteria.
             *
             * @param mp The motion planner to link to this stopping criteria.
             */
            void link_motion_planner(motion_planner_t* mp);

            /**
             * Link a goal to this set of criteria.
             * 
             * @brief Link a goal to this set of criteria.
             *
             * @param new_goal The goal to link to this stopping criteria.
             */
            void link_goal(util::goal_t* new_goal);

            /**
             * Get statistics on this stopping criteria.
             * 
             * @brief Get statistics on this stopping criteria.
             *
             * @param stats The vector of statistics from individual criteria returned by reference.
             */
            void get_statistics(std::vector<util::statistics_t*>& stats);

            /**
             * Get all criteria which are satisfied.
             * 
             * @brief Get all criteria which are satisfied.
             *
             * @return A vector containing all individual \ref criterion_t objects which are satisfied.
             */
            const std::vector<criterion_t*>& get_satisfied_criterion();

            /**
             * @brief Enumeration indicating how to check internal criteria.
             */
            enum criteria_check_type_t
            {

                ALL_CRITERIA = 1,
                SINGLE_CRITERION = 2,
            };

            /**
             * Set how the criteria checks for satisfaction.
             * 
             * @brief Set how the criteria checks for satisfaction.
             *
             * @param new_type Input enum indicating whether to check ALL or a SINGLE criterion.
             */
            void set_criteria_check_type(criteria_check_type_t new_type);

            /**
             * Link existing interruption criteria to this criteria class.
             * 
             * @brief Link existing interruption criteria to this criteria class.
             *
             * @param interruption_set The set of interruption criteria to check.
             */
            void link_interruption_criteria(const std::vector<criterion_t*>& interruption_set);

            /**
             * Add a new interruption-based criterion to this criteria class.
             * 
             * @brief Add a new interruption-based criterion to this criteria class.
             *
             * @param new_criterion The new interruption-based criterion to add.
             */
            void add_interruption_criterion(criterion_t* new_criterion);

            /**
             * Add a new criterion to this criteria class.
             * 
             * @brief Add a new criterion to this criteria class.
             *
             * @param new_criterion The new criterion to add.
             */
            void add_criterion(criterion_t* new_criterion);

            /**
             * @brief Exception raised when a stopping criterion is satisfied.
             */
            class stopping_criteria_satisfied : public std::runtime_error
            {

              public:

                stopping_criteria_satisfied(std::string msg)
                : std::runtime_error(msg){ };
            };

            /**
             * @brief Exception raised when an interruption from a stopping criterion occurs.
             */
            class interruption_criteria_satisfied : public std::runtime_error
            {

              public:

                interruption_criteria_satisfied(std::string msg)
                : std::runtime_error("Interruption Criterion satisfied: \n" + msg){ };
            };

            friend std::ostream& operator<<(std::ostream&, const stopping_criteria_t&);

          protected:
            /** @brief This stopping criteria's linked motion planner. */
            motion_planner_t* linked_motion_planner;

            /** @brief The type of check for criteria satisfaction: ALL or SINGLE. */
            criteria_check_type_t criteria_check_type;

            /** @brief All individual \ref criterion_t classes which comprise this criteria. */
            std::vector<criterion_t*> stopping_criteria;
            /** @brief All interruption-based criteria in this class. */
            std::vector<criterion_t*> interruption_criteria;

            /** @brief All criteria which have been satisfied. */
            std::vector<criterion_t*> satisfied_criterion;
        };

        /**
         * @brief Stream output operator for stopping criteria.
         */
        std::ostream& operator<<(std::ostream&, const stopping_criteria_t&);

    }
}

#endif

