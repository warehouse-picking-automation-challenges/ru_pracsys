/**
 * @file null_response_simulator.hpp
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

#ifndef PRX_NULL_RESPONSE_SIMULATOR_HPP
#define	PRX_NULL_RESPONSE_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/simulators/simulator.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * This is a variation of the simulator for null responses for the 
         * systems that are in collision. The simulator will check for collisions,
         * it can report them but will not do anything for these systems. For example
         * \ref collision_stop_simulator_t will respond after a collision and will stop
         * the systems before the collision. 
         * 
         * @brief <b> A null response simulator for the systems in collision. </.b>
         * 
         * @author Athanasios Krontiris
         * 
         */
        class null_response_simulator_t : public simulator_t
        {

          public:
            null_response_simulator_t();
            virtual ~null_response_simulator_t();

            /** @copydoc simulator_t::init(const util::parameter_reader_t* , const util::parameter_reader_t*)*/
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** 
             * This simulator will propagate the systems, but the response to the 
             * collisions between the systems will be nothing. 
             * 
             * @copydoc simulator_t::propagate_and_respond()     
             */
            virtual void propagate_and_respond();
        };

    }
}

#endif

