/**
 * @file integration_plant.hpp 
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

#ifndef INTEGRATION_PLANT_HPP
#define	INTEGRATION_PLANT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/simulation/integrators/integrator.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * A class for plants that can be integrated. Each plant if you want to simulate it, 
         * needs to extend and implement a \ref integration_plant_t in order to implement the 
         * integration functions. 
         * 
         * @brief <b> A plant that can be integrated. </b>
         * 
         * @author Andrew Dobson
         */
        class integration_plant_t : public plant_t
        {

          public:

            integration_plant_t();

            virtual ~integration_plant_t();

            /** @copydoc system_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*)*/
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** 
             * @copydoc system_t::propagate(const double)
             * 
             * @note This implementation works by using the concrete system's \ref integration_plant_t::update_derivative
             * with a \ref integrator_t to integrate the ordinary differential equation from the current state to the next.
             */
            virtual void propagate(const double simulation_step = 0);


            /**
             * @copydoc system_t::verify()
             *
             * Adds checks for the derivative function and integrator.
             */
            virtual void verify() const;

          protected:

            /**
             * Concrete subclasses of this class must implement this by taking the current state and control and putting
             * the calculated derivative into the passed \ref state_t.
             * 
             * @param result The output location for the updated derivative.
             */
            virtual void update_derivative(state_t * const result) = 0;

          protected:

            /**
             * A \ref boost::function wrapper around \ref integration_plant_t::update_derivative.
             */
            const integrator_t::derivative_function_t derivative_function;

            /**
             * The integrator to use when \ref integration_plant_t::propagate is called.
             */
            integrator_t* integrator;

        };

    }
}

#endif	

