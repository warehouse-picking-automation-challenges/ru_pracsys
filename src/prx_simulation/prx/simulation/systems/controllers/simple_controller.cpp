/**
 * @file simple_controller.cpp 
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

#include "prx/simulation/systems/controllers/simple_controller.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie

namespace prx
{
    using namespace util;
    namespace sim
    {

        simple_controller_t::simple_controller_t() : controller_t() { }

        simple_controller_t::~simple_controller_t() { }

        void simple_controller_t::add_system(const std::string& path, system_ptr_t system)
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            PRX_DEBUG_S("+++ ADD system : name : " << name << "  subpath: " << subpath);
            if( name.empty() )
                throw invalid_path_exception("Name is empty");
            else if( subpath.empty() )
            {
                PRX_ERROR_S("Trying to add system to a simple controller.  Throwing exception...");
                throw simple_add_remove_exception();
            }
            else if( !subpath.empty() )
            {
                if( subsystems.find(name) != subsystems.end() )
                    subsystems[name]->add_system(subpath, system);
            }
            else
                throw invalid_path_exception(path);

            construct_spaces();
            verify();
        }

        void simple_controller_t::remove_system(const std::string& path)
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            PRX_DEBUG_S("--- REMOVE system : name : " << name << "  subpath : " << subpath);
            if( subsystems.find(name) != subsystems.end() )
                if( subpath.empty() )
                {
                    PRX_ERROR_S("Trying to remove system from a simple controller.  Throwing exception...");
                    throw simple_add_remove_exception();
                }
                else
                {
                    try
                    {
                        subsystems[name]->remove_system(subpath);
                    }
                    catch( removal_exception e )
                    {
                        subsystems.erase(name);
                    }
                }

            else
                throw invalid_path_exception("The system " + name + " does not exist in the path " + pathname);

            construct_spaces();
            verify();
        }

        void simple_controller_t::propagate(const double simulation_step)
        {
            if( active )
            {
                subsystems.begin()->second->propagate(simulation_step);
            }
        }

        void simple_controller_t::compute_control()
        {
            if( active )
            {
                subsystems.begin()->second->compute_control();
            }
        }

        void simple_controller_t::update_phys_configs(config_list_t &configs, unsigned& index) const
        {
            if( subsystems.begin()->second->is_active() || simulation::update_all_configs )
                subsystems.begin()->second->update_phys_configs(configs, index);
        }

        void simple_controller_t::update_phys_geoms(geom_map_t &geoms) const
        {
            subsystems.begin()->second->update_phys_geoms(geoms);
        }

        void simple_controller_t::verify() const
        {
            // Check our spaces to make sure they exist
            //    PRX_ASSERT(input_control_space != NULL);
            PRX_ASSERT(output_control_space != NULL);
            PRX_ASSERT(state_space != NULL);

            // Simple controllers must have one subsystem
            PRX_ASSERT(subsystems.size() == 1);

            // Verify validity of our spaces
            if( input_control_space != NULL )
                input_control_space->verify();
            output_control_space->verify();
            state_space->verify();
            if( controller_state_space )
                controller_state_space->verify();

            hash_t<std::string, system_ptr_t >::const_iterator subsystems_const_iter;

            // Call our subsystems verify
            subsystems.begin()->second->verify();

            // Check to make sure our output_control_space == subsytems->get_control_space()
            std::string space_name;
            const space_t* cspace;
            cspace = subsystems.begin()->second->get_control_space();
            if( cspace != NULL )
                space_name = cspace->get_space_name();

            // Throw an exception if we do not match control spaces
            if( std::strcmp(space_name.c_str(), output_control_space->get_space_name().c_str()) )
            {
                PRX_ERROR_S("Output control space does not match subsystem's input control space");
                throw space_mismatch_exception();
            }
        }

        system_graph_t::directed_vertex_t simple_controller_t::update_system_graph(system_graph_t& graph)
        {
            system_graph_t::directed_vertex_t v = graph.add_node(pathname, false);

            system_graph_t::directed_vertex_t new_v = subsystems.begin()->second->update_system_graph(graph);
            graph.add_edge(v, new_v, subsystems.begin()->first, subsystems.begin()->second);

            return v;
        }

        void simple_controller_t::update_vis_info() const
        {
            subsystems.begin()->second->update_visualization_info();
        }
    }
}

