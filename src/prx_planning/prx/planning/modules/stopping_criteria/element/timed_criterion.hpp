/**
 * @file timed_criterion.hpp 
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

#ifndef PRX_TIMED_CRITERION_HPP
#define PRX_TIMED_CRITERION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/modules/stopping_criteria/element/criterion.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

namespace prx
{
    namespace util
    {
        class statistics_t;
    }

    namespace plan
    {

        /**
         * @anchor timed_criterion_t
         *
         * This criterion keeps track of how much time has passed and is satisfied after
         * reaching a time limit.  
         *
         * @brief <b> Criterion based on running time. </b>
         *
         * @author Andrew Kimmel
         */
        class timed_criterion_t : public criterion_t
        {

          public:
            timed_criterion_t();
            ~timed_criterion_t();

            /**
             * @copydoc criterion_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @copydoc criterion_t::criterion_check()
             */
            virtual bool criterion_check();
            /**
             * @copydoc criterion_t::print_statistics()
             */
            virtual std::string print_statistics();

            /**
             * @brief Set a new time limit.
             *
             * @param new_time_limit The new time limit to use for the criterion.
             */
            void set_time_limit(double new_time_limit);

            /**
             * @brief Get the current total time for this criterion.
             *
             * @return The time clocked by this criterion.
             */
            double get_time();
            /**
             * @brief Get the amount of time before criterion is satisfied.
             *
             * @return The amount of time before the criterion becomes satisfied.
             */
            double get_remaining_time();
            /**
             * @copydoc criterion_t::reset()
             */
            virtual void reset();

          protected:

          private:
            /** @brief The time limit before the criterion is satisfied. */
            double time_limit;
            /** @brief The internal timer used to keep track of the time. */
            util::sys_clock_t timer;
        };

    }
}

#endif
