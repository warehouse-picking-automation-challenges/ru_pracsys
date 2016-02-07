/**
 * @file euler_integrator.hpp 
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

#ifndef PRACSYS_EULER_INTEGRATOR_HPP
#define	PRACSYS_EULER_INTEGRATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/integrators/integrator.hpp"

#include <boost/function.hpp>

namespace prx
{
    namespace sim
    {

        /**
         * Performs Euler integration for fast and simple propagation of systems.
         * 
         * @brief <b> Performs Euler integrations.</b>
         * 
         * @author Athanasios Krontiris
         */
        class euler_integrator_t : public integrator_t
        {

          public:

            euler_integrator_t();
            ~euler_integrator_t();

            /** @copydoc integrator_t::init_space(const util::space_t* const) */
            virtual void init_space(const util::space_t * const space);

            /** @copydoc integrator_t::integrate(const derivative_function_t&, double)*/
            virtual void integrate(const derivative_function_t& derivative, double t = 0);

            /** @copydoc integrator_t::verify() const */
            virtual void verify() const;

          private:
            /**
             * Helper variable to maintain the derivative of the state.
             * 
             * @brief Helper variable to maintain the derivative of the state.
             */
            state_t* deriv;
        };




    }
}

#endif

