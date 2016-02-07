/**
 * @file collision_checker.cpp
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

#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <boost/range/adaptor/map.hpp> //adaptors
#include <boost/tuple/tuple.hpp> // boost::tie


namespace prx
{
    using namespace util;

    namespace sim
    {

        pluginlib::ClassLoader<collision_checker_t> collision_checker_t::loader("prx_simulation", "prx::sim::collision_checker_t");

        collision_checker_t::collision_checker_t()
        {
            colliding_bodies_list = new vector_collision_list_t();
        }

        void collision_checker_t::link_collision_list(collision_list_t* list)
        {
            collision_list = list;
        }

        void collision_checker_t::set_configurations(const config_list_t& map)
        {
            foreach(const config_list_element_t& name, map)
            set_configuration(name.first, name.second);
        }

        void collision_checker_t::add_bodies(const geom_map_t& map, hash_t<std::string, plant_t*>& bodies)
        {

            foreach(std::string name, map | boost::adaptors::map_keys)
            {
                std::string system_name;
                std::string rigid_body_name;
                boost::tie(system_name, rigid_body_name) = reverse_split_path(name);
                add_body(name, map[name], bodies[system_name]);
            }
        }

        bool collision_checker_t::in_collision( collision_list_t* list )
        {
            PRX_WARN_S("Unimplemented Virtual Function: collision_checker_t::in_collision( collision_list_t* ) : returning collision!");
            return true;
        }

        collision_list_t* collision_checker_t::colliding_bodies( collision_list_t* list )
        {
            PRX_WARN_S("Unimplemented Virtual Function: collision_checker_t::colliding_bodies( collision_list_t* ) : returning NULL!");
            return NULL;
        }

        pluginlib::ClassLoader<collision_checker_t>& collision_checker_t::get_loader()
        {
            return loader;
        }

    }
}



