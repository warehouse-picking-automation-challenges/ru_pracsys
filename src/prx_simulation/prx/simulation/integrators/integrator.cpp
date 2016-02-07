/**
 * @file integrator.cpp
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

#include "prx/simulation/integrators/integrator.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_list_macros.h>

namespace prx
{

    using namespace util;

    namespace sim
    {

        pluginlib::ClassLoader<integrator_t> integrator_t::loader("prx_simulation", "prx::sim::integrator_t");

        integrator_t::integrator_t()
        {
            sim_step = 0.017;
        }

        integrator_t::~integrator_t() { }

        void integrator_t::init_space(const space_t* space)
        {
            plant_state_space = space;
        }

        void integrator_t::init(const parameter_reader_t * const reader)
        {
            if( reader->has_attribute("simulation_step") )
                sim_step = reader->get_attribute_as<double>("simulation_step");
        }

        void integrator_t::verify() const
        {
            if( plant_state_space == NULL )
                throw invalid_integrator_exception("No plant_state_space has been set");
        }

        pluginlib::ClassLoader<integrator_t>& integrator_t::get_loader()
        {
            return loader;
        }

    }
}












