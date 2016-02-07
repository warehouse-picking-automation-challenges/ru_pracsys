/**
 * @file iteration_criterion.hpp 
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

#ifndef PRX_ITERATION_CRITERION_HPP
#define PRX_ITERATION_CRITERION_HPP

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
         * @anchor iteration_criterion_t
         *
         * This simple criterion checks an iteration limit.  If the iteration count has
         * surpassed some maximum threshold, then the criterion is satisfied.
         *
         * @brief <b> A criterion which checks an iteration counter. </b>
         *
         * @author Andrew Kimmel
         */
        class iteration_criterion_t : public criterion_t
        {

          public:
            iteration_criterion_t();
            ~iteration_criterion_t();

            /**
             * @copydoc criterion_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * @brief Set a new iteration limit.
             *
             * @param new_max The new iteration limit.
             */
            virtual void set_max_iterations(int new_max);
            virtual int get_max_iterations();
            /**
             * @copydoc criterion_t::criterion_check()
             *
             * @note Checks if iteration_counter >= max_iterations.
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
            /** @brief Iteration limit. */
            int max_iterations;
            /** @brief Current iteration. */
            int iteration_counter;

        };

    }
}

#endif