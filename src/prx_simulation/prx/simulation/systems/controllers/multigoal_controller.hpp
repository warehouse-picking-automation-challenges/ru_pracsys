/**
 * @file multigoal_controller.hpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_MULTIGOAL_CONTROLLER_HPP
#define PRX_MULTIGOAL_CONTROLLER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/simulation/control.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/any.hpp>
#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>
#include <vector>



namespace prx
{
    namespace sim
    {

        /**
         * A controller which takes a sequence of multigoals from input and 
         * guides its child system between them in a predefined pattern. Each multigoal 
         * is a configuration in the state space of the child system under the 
         * controller in the system tree.
         * See the test experiment in prx_input/experiments/multigoal_controller/ 
         * for usage details.
         * 
         * Currently implemented modes are:
         * 0. Forward loop -- child system travels to each multigoal in the order it's
         * specified, returning to the first after reaching the end.
         * 1. Forward/reverse -- child travels to each multigoal in order and 
         * then returns the way it came, going back through all multigoals in 
         * reverse.
         * 
         * @brief <b>A controller which takes a sequence of multigoals from input and guides a system between them in a predefined pattern.</b>
         * 
         * @author Andrew Kimmel
         */
        class multigoal_controller_t : public simple_controller_t
        {

          public:
            /**
             * Initializes internal variables and sets up the controller's state space with the time parameter.
             * 
             * @brief Initializes internal variables and sets up the controller's state space with the time parameter.
             */
            multigoal_controller_t();

            /** @copydoc system_t::~system_t() */
            virtual ~multigoal_controller_t();

            /**
             * Reads the list of multigoals from input and creates the sequence of plans 
             * which will be used to control the child system.
             * 
             * Also does standard controller_t::init()
             * 
             * @brief Initializes the multigoal_controller_t 
             * @param reader Used to initialize the controller
             * @param template_reader Used to initialize the controller from template files
             */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /**
             * Calls \c propagate to its single subsystem.
             * 
             * @brief Calls \c propagate to its single subsystem.
             */
           // virtual void propagate(const double simulation_step = 0);

            /**
             * Calls \c compute_control to its single subsystem.
             * 
             * @brief \c Calls compute_control to its single subsystem.
             */
            virtual void compute_control();

          protected:
            //The list of multigoals that the child system should move between.
            std::vector<util::space_point_t*> multigoals;

//            //The plan to follow for traversing the multigoals.
//            plan_t loop;
            
            //Index for the next goal
            unsigned int goal_index;

            //The cached plan lengths for quick time checks.
            double plan_length;

            //The current simulation time.
            double simtime;
            
            //radius of reach
            double goal_radius;

            //Child system information.
            system_ptr_t child_system;
           
            const util::space_t *child_state_space;
            const util::space_t *child_control_space;
            
            // Current state of child
            state_t* current_state;
            control_t* current_control;
        };
    }
}


#endif