/**
 * @file collision_stop_simulator.cpp
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

#include "prx/simulation/simulators/collision_stop_simulator.hpp"
#include <pluginlib/class_list_macros.h>
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/tuple/tuple.hpp>
#include <boost/range/adaptor/map.hpp> // boost::tie

PLUGINLIB_EXPORT_CLASS(prx::sim::collision_stop_simulator_t, prx::sim::simulator_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        collision_stop_simulator_t::collision_stop_simulator_t()
        {
            prev_state = NULL;
            state = NULL;
        }

        collision_stop_simulator_t::~collision_stop_simulator_t()
        {
            if( prev_state != NULL )
                state_space->free_point(prev_state);
            if( state != NULL )
                state_space->free_point(state);
        }

        void collision_stop_simulator_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {

            simulator_t::init(reader, template_reader);
            prev_state = state_space->alloc_point();
            state = state_space->alloc_point();
        }

        void collision_stop_simulator_t::propagate_and_respond()
        {
            state_space->copy_to_point(prev_state);

            //propagate all the systems in the simulator.
            router_t::propagate(simulation::simulation_step);
            collision_list_t* in_collision_list = simulator_t::get_colliding_bodies();
            bool collided = false;

            if( in_collision_list->size() > 0 )
            {
                collided = true;
                state_space->copy_to_point(state);
            }

            foreach(collision_pair_t p, in_collision_list->get_body_pairs())
            {
                set_previous_state(p.first);
                set_previous_state(p.second);
                PRX_DEBUG_COLOR("in collision : " << p.first << "  -> " << p.second, PRX_TEXT_MAGENTA);
            }

            if( collided )
                push_state(state);
        }

        void collision_stop_simulator_t::set_previous_state(const std::string& path)
        {
            std::string name_to_check;
            std::string name;
            std::string subpath;


            // remove the /geometry_name from the end of the path;
            boost::tie(name_to_check, subpath) = reverse_split_path(path);
            if( plants.find(name_to_check) != plants.end() )
            {
                std::pair<unsigned, unsigned> interval = system_intervals[(system_t*)plants[name_to_check]];
                for( unsigned i = interval.first; i < interval.second; i++ )
                {
                    state->at(i) = prev_state->at(i);
                }
            }
        }
    }
}

