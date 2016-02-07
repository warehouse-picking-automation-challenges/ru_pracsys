/**
 * @file temporal_validity_checker.hpp 
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

#ifndef PRX_TEMPORAL_VALIDITY_CHECKER_HPP
#define PRX_TEMPORAL_VALIDITY_CHECKER_HPP

#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @anchor temporal_validity_checker_t
         *
         * The temporal validity checker relies on the world model to 
         * inform a trajectory if during propagation a collision occured.
         * This is necessary for physics simulation, where the trajectory 
         * will always consist of states that are not in collision, but may have
         * collided during simulaiton fo the trajectory.
         *
         * @brief <b> Queries informed trajectories about collisions. </b>
         *
         * @author Zakary Littlefield
         */
        class temporal_validity_checker_t : public validity_checker_t
        {

          public:

            temporal_validity_checker_t() : validity_checker_t(){ }

            virtual ~temporal_validity_checker_t(){ }

            /**
             * @copydoc validity_checker_t::is_valid()
             */
            virtual bool is_valid(const sim::state_t* point);
            
            /**
             * @brief Validity check for an entire trajectory.
             *
             * @param input The input trajectory to be checked for validity.
             */
            virtual bool is_valid(const sim::trajectory_t& input);
        };

    }
}


#endif