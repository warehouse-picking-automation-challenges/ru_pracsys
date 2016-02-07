/**
 * @file no_collision_simulator.hpp
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

#ifndef PRX_NO_COLLISION_SIMULATOR_HPP
#define	PRX_NO_COLLISION_SIMULATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/simulators/simulator.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * This is a different variation of a simulator. This simulator will not check for 
         * collisions at all. It does not contain a \ref collision_checker_t. If you care 
         * about collisions you should use another variation of simulator. This version is 
         * better when you need to simulate a lot of systems that you do not care about collisions.
         * 
         * @brief <b>A simulator for simulating the systems, without checking for collisions. </b>
         * 
         * @author Athanasios Krontiris
         */
        class no_collision_simulator_t : public simulator_t
        {

          public:
            no_collision_simulator_t();
            virtual ~no_collision_simulator_t();

            /**
             * @copydoc simulator_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*)
             * 
             * @note This simulator will not initialize a collision checker. 
             */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /**      
             * 
             * @copydoc simulator_t::propagate_and_respond()      
             * 
             * @note This simulator will not check for collisions. 
             */
            virtual void propagate_and_respond();

            /**
             * Overwrites the default implementation of the \ref simulator_t::in_collision() 
             * so as will not checks for collisions. This simulator does not have 
             * \ref collision_checker_t.
             * 
             * @brief This function will just warn you that there is no \ref collision_checker_t.
             * 
             * @return Always False.
             */
            bool in_collision();

            /**
             * This simulator does not have \ref collision_checker_t, so this function 
             * will just warn you for that case. 
             * 
             * @brief This function will just warn you that there is no \ref collision_checker_t.
             * 
             * @param collision_list The list we want to check for collisions. This simulator
             * will not use this list because does not have \ref collision_checker_t for collisions.
             */
            void link_collision_list(collision_list_t* collision_list);

            /**
             * This simulator does not have \ref collision_checker_t, so this function 
             * will just warn you for that case. 
             * 
             * @brief This function will just warn you that there is no \ref collision_checker_t.
             * 
             * @return Always NULL.
             */
            collision_list_t* get_colliding_bodies();

            /**
             * It will add an obstacle in the obstacle list of the simulator, but it will avoid
             * to add the obstacle to the \ref collision_checker_t. This simulator does not have
             * \ref collision_checker_t.
             * 
             * @brief Adds an obstacle into the simulation. Avoids to add the obstacle in the 
             * \ref collision_checker_t.
             * 
             * @param name A slash-delimited path name of the obstacle that we want to add. 
             * (eg. simulator/obstacles/box)
             * @param obstacle Is the system pointer for the obstacle that we want to add in 
             * the simulation.
             */
            void add_obstacle(const std::string& name, system_ptr_t obstacle);

            /**
             * It will remove an obstacle from the simulation, but it will avoid
             * to remove the obstacle from the \ref collision_checker_t. This simulator does not have
             * \ref collision_checker_t.
             * 
             * @brief Removes an obstacle from the simulation. Avoids to remove the obstacle from the 
             * \ref collision_checker_t.
             *  
             * @param name Is the slash-delimited name of the obstacle that we want to remove from
             * the simulator. (eg. simulator/obstacles/box)
             */
            void remove_obstacle(const std::string& name);

            /**
             * Verifies if the simulator is valid. It will not check if the \ref collision_checker_t 
             * is valid because this simulator does not have \ref collision_checker_t.
             */
            void verify() const;

        };

    }
}

#endif
