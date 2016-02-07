/**
 * @file rk4_integrator.cpp
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

#include "prx/simulation/integrators/rk4_integrator.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::rk4_integrator_t, prx::sim::integrator_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        rk4_integrator_t::rk4_integrator_t()
        {
            deriv1 = NULL;
            deriv2 = NULL;
            deriv3 = NULL;
            deriv4 = NULL;
            initial = NULL;
        }

        rk4_integrator_t::~rk4_integrator_t()
        {
            plant_state_space->free_point(deriv1);
            plant_state_space->free_point(deriv2);
            plant_state_space->free_point(deriv3);
            plant_state_space->free_point(deriv4);
            plant_state_space->free_point(initial);
        }

        void rk4_integrator_t::init_space(const space_t* space)
        {
            integrator_t::init_space(space);
            deriv1 = plant_state_space->alloc_point();
            deriv2 = plant_state_space->alloc_point();
            deriv3 = plant_state_space->alloc_point();
            deriv4 = plant_state_space->alloc_point();
            initial = plant_state_space->alloc_point();
        }

        void rk4_integrator_t::integrate(const derivative_function_t& derivative, double t)
        {
            //    if( t == 0 )
            //        t = sim_step;

            plant_state_space->copy_to_point(initial);

            derivative(deriv1);
            plant_state_space->integrate(initial, deriv1, t / 2.0);

            derivative(deriv2);
            plant_state_space->integrate(initial, deriv2, t / 2.0);

            derivative(deriv3);
            plant_state_space->integrate(initial, deriv3, t / 2.0);

            derivative(deriv4);

            plant_state_space->integrate(initial, deriv1, t / 6.0);
            plant_state_space->integrate(deriv2, t / 3.0);
            plant_state_space->integrate(deriv3, t / 3.0);
            plant_state_space->integrate(deriv4, t / 6.0);
        }

        void rk4_integrator_t::verify() const
        {
            integrator_t::verify();
            if( deriv1 == NULL || deriv2 == NULL || deriv3 == NULL || deriv4 == NULL || initial == NULL )
                throw invalid_integrator_exception("The Runge-Kutta integrator has not initialized the points.");

        }

    }
}

