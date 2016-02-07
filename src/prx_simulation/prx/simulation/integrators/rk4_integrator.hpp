/**
 * @file rk4_integrator.hpp
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

#ifndef PRX_RK4_INTEGRATOR_HPP
#define	PRX_RK4_INTEGRATOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/integrators/integrator.hpp"

#include <boost/function.hpp>

namespace prx
{
    namespace sim
    {

        /**
         * Performs Runge-Kutta integration for propagation of systems.
         * 
         * @brief <b> Performs Runge-Kutta integrations.</b>
         * 
         * @author Athanasios Krontiris
         */
        class rk4_integrator_t : public integrator_t
        {

          public:

            rk4_integrator_t();
            virtual ~rk4_integrator_t();

            /** @copydoc integrator_t::init_space(const util::space_t* const) */
            virtual void init_space(const util::space_t * const space);

            /** @copydoc integrator_t::integrate(const derivative_function_t&, double); */
            virtual void integrate(const derivative_function_t& derivative, double t = 0);

            /** @copydoc integrator_t::verify() const */
            virtual void verify() const;

          private:
            /**
             * Helper variable to maintain the initial state.
             * 
             * @brief Helper variable to maintain the initial state.
             */
            state_t* initial;
            /**
             * Helper variable to keep the first derivative of the integration.
             * 
             * @brief Helper variable to keep the first derivative of the integration.
             */
            state_t* deriv1;
            /**
             * Helper variable to keep the second derivative of the integration.
             * 
             * @brief Helper variable to keep the second derivative of the integration.
             */
            state_t* deriv2;
            /**
             * Helper variable to keep the third derivative of the integration.
             * 
             * @brief Helper variable to keep the third derivative of the integration.
             */
            state_t* deriv3;
            /**
             * Helper variable to keep the forth derivative of the integration.
             * 
             * @brief Helper variable to keep the forth derivative of the integration.
             */
            state_t* deriv4;

        };

    }
}

#endif

