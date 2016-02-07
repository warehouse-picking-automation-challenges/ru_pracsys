/**
 * @file succeeded_criterion.hpp
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

#ifndef PRX_SUCCEEDED_CRITERION_HPP
#define PRX_SUCCEEDED_CRITERION_HPP

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
         * @anchor succeeded_criterion_t
         *
         * This simple criterion asks the motion planner if it thinks it
         * has succeeded by calling its succeeded() function.
         *
         * @brief <b> A criterion which checks an succeeded counter. </b>
         *
         * @author Andrew Kimmel
         */
        class succeeded_criterion_t : public criterion_t
        {

          public:
            succeeded_criterion_t();
            ~succeeded_criterion_t();

            /**
             * @copydoc criterion_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @copydoc criterion_t::criterion_check()
             *
             * @note Checks if succeeded_counter >= max_succeededs.
             */
            virtual bool criterion_check();
            /**
             * @copydoc criterion_t::print_statistics()
             */
            virtual std::string print_statistics();
            /**
             * @copydoc criterion_t::reset()
             */
            virtual void reset();

          protected:

          private:

        };

    }
}

#endif
