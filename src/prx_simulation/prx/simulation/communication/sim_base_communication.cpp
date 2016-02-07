/**
 * @file sim_base_communication.cpp 
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

#include "prx/simulation/communication/sim_base_communication.hpp"

namespace prx
{
    namespace sim
    {

        pluginlib::ClassLoader<sim_base_communication_t> sim_base_communication_t::loader("prx_simulation", "prx::sim::sim_base_communication_t");

        void sim_base_communication_t::link_application(application_t* parent_app)
        {
            app = parent_app;
        }

        pluginlib::ClassLoader<sim_base_communication_t>& sim_base_communication_t::get_loader()
        {
            return loader;
        }

    }
}