/**
 * @file integrator.hpp 
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

#ifndef INTEGRATOR_HPP
#define	INTEGRATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/state.hpp"

#include <boost/function.hpp>
#include <pluginlib/class_loader.h>

namespace prx
{
    namespace sim
    {

        /**
         * Performs the numerical integration of a \ref util::space_point_t for a fixed time step given a
         * derivative function. Each \ref plant_t has to implement its own derivative function for the
         * integration. 
         * 
         * @brief <b> Performs the numerical integration of a \ref util::space_point_t </b>
         * 
         * @author Andrew Dobson
         */
        class integrator_t
        {

          public:

            integrator_t();
            virtual ~integrator_t();

            /**
             * Initializes the integrator from the given parameters.
             * 
             * @brief Initializes from the given parameters.
             * 
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
             */
            virtual void init(const util::parameter_reader_t * const reader);

            /**
             * Initializes the space of the plant so as the integrator can allocate  
             * space_points for the integration. 
             * 
             * @brief Initializes the space of the controlled plant.
             * 
             * @param space The plants \ref util::space_t.
             */
            virtual void init_space(const util::space_t * const space);

            /**
             * A function that computes the derivative of a given point in a differentiable space.
             * 
             * @brief A function that computes the derivative of a given point in a differentiable space.
             */
            typedef boost::function< void (state_t *) > derivative_function_t;

            /**
             * Integrates the \ref plant_t given the function that will compute the derivative of the 
             * controlled \ref plant_t and the time for the integration.
             * 
             * @brief Integrates the \ref plant_t.
             * 
             * @param derivative A function that calculates the derivative of a point on the differential space.
             * @param t The time the function will integrate the \ref plant_t.
             */
            virtual void integrate(const derivative_function_t& derivative, double t = 0) = 0;

            /** 
             * Makes sure that the \ref integrator_t is valid, after the initialization.
             * 
             * @brief Makes sure that the \ref integrator_t is valid.
             * 
             * @exception integrator_t::invalid_integrator_exception Throws an \ref integrator_t::invalid_integrator_exception if the 
             * \ref integrator_t is not valid.
             */
            virtual void verify() const;

            /**
             * It returns a pluginlib class loader for the current class.
             * 
             * @brief It returns a plugin lib class loader for the current class.
             * 
             * @return The pluginlib loader for the current class.
             */
            static pluginlib::ClassLoader<integrator_t>& get_loader();

            /**
             * Will be thrown if a \ref integrator_t verification failed.
             * 
             * @brief Will be thrown if a \ref integrator_t verification failed.
             */
            class invalid_integrator_exception : public std::runtime_error
            {

              public:

                invalid_integrator_exception(const std::string& msg) : std::runtime_error(msg){ };
            };


          protected:

            /** 
             * Simulation step used for the integration.
             * 
             * @brief Simulation step used for the integration.
             */
            double sim_step;
            /** 
             * The state of the \ref plant_t that implements the integrator.
             * 
             * @brief The state of the \ref plant_t that implements the integrator.
             */
            const util::space_t* plant_state_space;

          private:
            /** 
             * The pluginlib loader for the \ref integrator_t class.
             * 
             * @brief The pluginlib loader for the \ref integrator_t class.
             */
            static pluginlib::ClassLoader<integrator_t> loader;
        };

    }
}

#endif
