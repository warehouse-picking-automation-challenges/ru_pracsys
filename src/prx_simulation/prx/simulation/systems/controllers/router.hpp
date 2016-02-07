/**
 * @file router.hpp
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

#ifndef PRX_ROUTER_HPP
#define	PRX_ROUTER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/controllers/controller.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Router is a controller that will not compute new controls for the subsystems.
         * It will just pass to the subsystems the controls that will get from the controller above.
         * Because the control that will accept will be a composite control for all the subsystems
         * of the router, the router will split this controller into the parts that belongs to each
         * subsystems and it will pass to each system the correct part from the composite control.
         * 
         * @brief <b> Splits a composite control into its subsystem. </b>
         * 
         * @author Athanasios Krontiris, Zakary Littlefield
         */
        class router_t : public controller_t
        {

          public:
            router_t();
            virtual ~router_t();

            /**
             * Steers between a composite initial state to a composite goal state. Each system will get 
             * different length plans and this function will try to make all the plans for 
             * all different subsystems to be the same length.
             * 
             * @param start The composite initial state. Each subsystem has as its initial 
             * state a part of this composite initial state.
             * @param goal The composite goal state. Each subsystem has as its goal state 
             * a part of this composite goal state.     
             * @param result_plan The resulted composite plan for all the subsystems.
             * 
             * @brief Will steers between an initial to a goal state for all the subsystems.
             */
            virtual void steering_function(const state_t* start, const state_t* goal, plan_t& result_plan);

            /** @copydoc system_t::append_contingency(plan_t&, double)*/
            virtual void append_contingency(plan_t& result_plan, double duration);

          protected:

            /**
             * Constructs the state and control spaces of the \ref router_t based on the subsystems.
             * 
             * @brief Constructs the state and control spaces of the \ref router_t based on the subsystems.     
             */
            virtual void construct_spaces();

            /** A hash maps that helps for the decomposition of the controls for the steering function */
            util::hash_t<std::string, std::pair<unsigned, unsigned> > control_map;

            /** A hash maps that helps for the decomposition of the states for the steering function */
            util::hash_t<std::string, std::pair<unsigned, unsigned> > state_map;
            /** 
             * The different plans that will be computed after the steers. Each
             * plant will have a different plan based on its initial and goal state.
             */
            util::hash_t<std::string, plan_t> subplans;
        };

    }
}

#endif
