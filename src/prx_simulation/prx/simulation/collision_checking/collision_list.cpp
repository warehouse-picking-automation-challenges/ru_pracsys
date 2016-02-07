/**
 * @file collision_list.cpp 
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



#include "prx/simulation/collision_checking/collision_list.hpp"

namespace prx
{
    namespace sim
    {

        pluginlib::ClassLoader<collision_list_t> collision_list_t::loader("prx_simulation", "prx::sim::collision_list_t");

        void collision_list_t::add_pair(const std::string& system1, const std::string& system2)
        {
            std::pair<std::string, std::string> pair = std::make_pair(system1, system2);
            add_new_pair(pair);
        }

        std::string collision_list_t::print() const
        {
            PRX_WARN_S("The current collision list does not support the function print()!");
            return "";
        }

        pluginlib::ClassLoader<collision_list_t>& collision_list_t::get_loader()
        {
            return loader;
        }

    }
}

