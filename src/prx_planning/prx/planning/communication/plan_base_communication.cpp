/**
 * @file plan_base_communication.cpp 
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

#include "prx/planning/communication/plan_base_communication.hpp"

namespace prx
{
    using namespace util;
    namespace plan
    {

        pluginlib::ClassLoader<plan_base_communication_t> plan_base_communication_t::loader("prx_planning", "prx::plan::plan_base_communication_t");

        void plan_base_communication_t::link_application(planning_application_t* parent_app)
        {
            planning_app = parent_app;
        }

        pluginlib::ClassLoader<plan_base_communication_t>& plan_base_communication_t::get_loader()
        {
            return loader;
        }

    }
}