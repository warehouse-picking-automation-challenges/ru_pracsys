/**
 * @file no_collision_simulator.cpp
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


#include "prx/simulation/simulators/no_collision_simulator.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/range/adaptor/map.hpp> //adaptors
#include <boost/tuple/tuple.hpp> // boost::tie

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::no_collision_simulator_t, prx::sim::simulator_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        no_collision_simulator_t::no_collision_simulator_t() { }

        no_collision_simulator_t::~no_collision_simulator_t() { }

        void no_collision_simulator_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            geom_map_t geom_map;
            config_list_t config_map;
            std::string obstacle_name;
            std::string template_name;

            simulation::simulation_step = parameters::get_attribute_as<double>("simulation_step", reader, template_reader);

            if( parameters::has_attribute("pathname", reader, template_reader) )
            {

                pathname = parameters::get_attribute("pathname", reader, template_reader);
                PRX_DEBUG_COLOR("Set pathname: " << pathname, PRX_TEXT_LIGHTGRAY);
            }

            if( parameters::has_attribute("obstacles", reader, template_reader) )
            {
                parameter_reader_t::reader_map_t obstacle_map = parameters::get_map("obstacles", reader, template_reader);

                foreach(const parameter_reader_t::reader_map_t::value_type key_value, obstacle_map)
                {
                    const parameter_reader_t* obstacle_template_reader = NULL;

                    if( key_value.second->has_attribute("template") )
                    {
                        template_name = key_value.second->get_attribute("template");
                        // TODO: Find a way to load templates under namespaces more cleanly.
                        obstacle_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);
                    }

                    obstacle_name = pathname + "/obstacles/" + key_value.first;
                    add_obstacle(obstacle_name, create_subsystem(obstacle_name, key_value.second, obstacle_template_reader));
                    delete obstacle_template_reader;
                }
            }

            if( parameters::has_attribute("subsystems", reader, template_reader) )
            {
                parameter_reader_t::reader_map_t subsystem_map = parameters::get_map("subsystems", reader, template_reader);

                foreach(const parameter_reader_t::reader_map_t::value_type key_value, subsystem_map)
                {
                    PRX_DEBUG_COLOR("Composite controller subsystem " << key_value.first, PRX_TEXT_GREEN);
                    const parameter_reader_t* subsystems_template_reader = NULL;

                    if( key_value.second->has_attribute("template") )
                    {
                        template_name = key_value.second->get_attribute("template");
                        PRX_DEBUG_COLOR("We have a template!:" << template_name, PRX_TEXT_CYAN);
                        // TODO: Find a way to load templates under namespaces more cleanly.
                        subsystems_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);

                    }
                    subsystems[key_value.first] = create_subsystem(pathname + "/" + key_value.first, key_value.second, subsystems_template_reader);
                    subsystem_names.push_back(key_value.first);
                    PRX_DEBUG_COLOR("set subsystem : " << key_value.first << "  under path : " << pathname + "/" + key_value.first, PRX_TEXT_GREEN);
                    if( subsystems_template_reader != NULL )
                        delete subsystems_template_reader;
                }
            }
            construct_spaces();

            simulator_state = state_space->alloc_point();

            update_phys_geoms(geom_map);
            unsigned index = 0;
            update_phys_configs(config_map, index);

            system_graph_t::directed_vertex_t v = update_system_graph(sys_graph);
            sys_graph.system_graph[v].name = "simulator";
            //    sys_graph.system_graph[v].system.reset(this);
            sys_graph.get_path_plant_hash(plants);


            //TODO: We should be constructing all system intervals in initialization (not just plants.)

            foreach(plant_t* plant, plants | boost::adaptors::map_values)
            {
                //find the place in the structure where this plant's state space exists
                std::pair<unsigned, unsigned> space_bounds = state_space->get_subspace(plant->get_state_space());
                if( space_bounds.first != space_bounds.second )
                {
                    system_intervals[(system_t*)plant] = space_bounds;
                }
            }

            verify();
        }

        bool no_collision_simulator_t::in_collision()
        {
            PRX_WARN_S("No Collision Simulator cannot call in_collision!");
            return false;
        }

        void no_collision_simulator_t::link_collision_list(collision_list_t* collision_list)
        {
            PRX_WARN_S("No Collision Simulator does not need a collision list!");
        }

        collision_list_t* no_collision_simulator_t::get_colliding_bodies()
        {
            PRX_WARN_S("No Collision Simulator does not have collision list for colliding_bodies!");
            return NULL;
        }

        void no_collision_simulator_t::add_obstacle(const std::string& name, system_ptr_t obstacle)
        {
            geom_map_t geom_map;
            config_list_t config_map;
            if( obstacles[name] == NULL )
            {
                PRX_DEBUG_COLOR("new obstacle : " << name, PRX_TEXT_GREEN);
                obstacles[name] = obstacle;
                obstacles[name]->update_phys_geoms(geom_map);
                unsigned index = 0;
                obstacles[name]->update_phys_configs(config_map, index);
            }
        }

        void no_collision_simulator_t::remove_obstacle(const std::string& name)
        {
            if( obstacles[name] != NULL )
            {
                obstacles.erase(name);
            }
        }

        void no_collision_simulator_t::verify() const
        {
            controller_t::verify();
        }

        void no_collision_simulator_t::propagate_and_respond()
        {
            //propagate all the systems in the simulator.
            router_t::propagate(simulation::simulation_step);
        }

    }
}
