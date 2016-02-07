/**
 * @file goal_criterion.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_GOAL_CRITERION_HPP
#define PRX_GOAL_CRITERION_HPP

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
        /**
         * @anchor goal_criterion_t
         *
         * This criterion uses a /ref util::goal_t in order to check for satisfaction.
         *
         * @brief <b> A goal based criterion </b>
         *
         * @author Andrew Kimmel
         */
        class goal_criterion_t : public criterion_t
        {

          public:

            goal_criterion_t();
            ~goal_criterion_t();

            /**
             * @copydoc criterion_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @copydoc criterion_t::criterion_check()
             *
             * @note Calls the linked /ref util::goal_t::satisfied() function
             */
            virtual bool criterion_check();

            /**
             * @copydoc criterion_t::link_goal()
             * @note Stores the new_goal into linked_goal
             */
            virtual void link_goal(util::goal_t* new_goal);

            /**
             * @copydoc criterion_t::reset()
             */
            virtual void reset();

          protected:



          private:
            /** @brief Used to check for goal satisfaction */
            util::goal_t* linked_goal;


        };

    }
}

#endif