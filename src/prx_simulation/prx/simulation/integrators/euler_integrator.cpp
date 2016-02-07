/**
 * @file euler_integrator.cpp
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

#include "prx/simulation/integrators/euler_integrator.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::euler_integrator_t, prx::sim::integrator_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        euler_integrator_t::euler_integrator_t() : integrator_t()
        {
            deriv = NULL;
        }

        euler_integrator_t::~euler_integrator_t()
        {
            plant_state_space->free_point(deriv);
        }

        void euler_integrator_t::init_space(const space_t * const space)
        {
            integrator_t::init_space(space);
            deriv = plant_state_space->alloc_point();
        }

        void euler_integrator_t::integrate(const derivative_function_t& derivative, double t)
        {
            derivative(deriv);
            plant_state_space->integrate(deriv, t);
        }

        void euler_integrator_t::verify() const
        {
            integrator_t::verify();

            if( deriv == NULL )
                throw invalid_integrator_exception("Euler integrator has not initialized the space points.");
        }

    }
}

