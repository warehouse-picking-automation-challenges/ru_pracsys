/**
 * @file ignore_list_validity_checker.hpp 
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

#ifndef PRX_IGNORE_LIST_VALIDITY_CHECKER_HPP
#define PRX_IGNORE_LIST_VALIDITY_CHECKER_HPP

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @anchor ignore_list_validity_checker_t
         *
         * This validity checker uses an internal \ref sim::collision_list_t to represent
         * pairs of bodies for which collision is acceptable.  For instance, we use this
         * validity checker to allow a physically simulated car's wheels to collide with
         * the ground plane, as most valid states of such a vehicle have this collision
         * occurring.
         *
         * @brief <b> Validity checker which ignores certain collisions. </b>
         */
        class ignore_list_validity_checker_t : public validity_checker_t
        {

          public:

            ignore_list_validity_checker_t() : validity_checker_t(){ }

            virtual ~ignore_list_validity_checker_t(){ }

            /**
             * @copydoc validity_checker_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
            /**
             * @copydoc validity_checker_t::is_valid()
             */
            virtual bool is_valid(const sim::state_t* point);

          protected:
            /** @brief A list containing collision pairs which do not cause invalid states. */
            sim::collision_list_t* ignore_list;
        };

    }
}


#endif