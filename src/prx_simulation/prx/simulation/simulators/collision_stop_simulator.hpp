/**
 * @file collision_stop_simulator.hpp
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

#ifndef PRX_COLLISION_STOP_SIMULATOR_HPP
#define	PRX_COLLISION_STOP_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/simulators/simulator.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * This is a variation of the simulator will response to the collisions 
         * of the systems. If a systems is in collision it will stop one step before 
         * the collision. 
         * 
         * @brief <b> A simulator that responses to the collisions </b>
         * 
         * @author Athanasios Krontiris
         * 
         */
        class collision_stop_simulator_t : public simulator_t
        {

          public:
            collision_stop_simulator_t();
            virtual ~collision_stop_simulator_t();

            /** @copydoc simulator_t::init( const util::parameter_reader_t* const) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc simulator_t::propagate_and_respond() */
            virtual void propagate_and_respond();

          protected:

            /** A hash map that keeps the collided systems from the last step.*/
            util::hash_t<std::string, bool> collided_systems;

            /** The previous state for all the systems in order to return the collided systems.*/
            state_t* prev_state;

            /** The current state of the simulation.*/
            state_t* state;

            /**
             * Sets the system under the location that the \c path specified, to the previous state, 
             * right before the collision.
             * 
             * @param path The slash-delimited path indicating the location of the system in the simulation tree.
             */
            void set_previous_state(const std::string& path);

          private:

        };

    }
}

#endif

