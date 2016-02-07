/**
 * @file null_response_simulator.cpp
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

#include "prx/simulation/simulators/null_response_simulator.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/utilities/spaces/space.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::null_response_simulator_t, prx::sim::simulator_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        null_response_simulator_t::null_response_simulator_t() { }

        null_response_simulator_t::~null_response_simulator_t() { }

        void null_response_simulator_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            simulator_t::init(reader, template_reader);
        }

        void null_response_simulator_t::propagate_and_respond()
        {
            //propagate all the systems in the simulator.
            router_t::propagate(simulation::simulation_step);
            collision_checker->in_collision();
        }

    }
}

