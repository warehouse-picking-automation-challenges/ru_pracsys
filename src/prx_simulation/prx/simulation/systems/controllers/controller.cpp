/**
 * @file controller.cpp 
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

#include "prx/simulation/systems/controllers/controller.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>

namespace prx
{
    using namespace util;
    namespace sim
    {

        controller_t::controller_t()
        {
            controller_state_space = NULL;
            state_space = NULL;
            input_control_space = NULL;
            output_control_space = NULL;
            computed_control = NULL;
        }

        controller_t::~controller_t()
        {

            foreach(double* mem, state_memory)
            {
                delete mem;
            }
            foreach(sensing_info_t* info, sensing_info)
            {
                delete info;
            }
            output_control_space->free_point(computed_control);
            delete input_control_space;
            delete output_control_space;
            delete state_space;
            if( controller_state_space )
                delete controller_state_space;
        }

        void controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            //This function should be called in concrete controllers after the template_reader has been read and initialized things.    
            std::string template_name = "";
            PRX_DEBUG_S("Init controller");
            if( parameters::has_attribute("subsystems", reader, template_reader) )
            {
                parameter_reader_t::reader_map_t subsystem_map = parameters::get_map("subsystems", reader, template_reader);

                foreach(const parameter_reader_t::reader_map_t::value_type key_value, subsystem_map)
                {
                    PRX_DEBUG_S("Controller subsystem " << key_value.first);

                    const parameter_reader_t* subsystems_template_reader = NULL;

                    if( key_value.second->has_attribute("template") )
                    {
                        template_name = key_value.second->get_attribute("template");

                        // TODO: Find a way to load templates under namespaces more cleanly.
                        subsystems_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);


                    }
                    //            PRX_ERROR_S("Template found with pathname: "<<pathname);
                    subsystems[key_value.first] = create_subsystem(pathname + "/" + key_value.first, key_value.second, subsystems_template_reader);
                    subsystem_names.push_back(key_value.first);
                    PRX_DEBUG_COLOR("set subsystem : " << key_value.first << "  under path : " << pathname + "/" + key_value.first, PRX_TEXT_GREEN);
                    if( subsystems_template_reader != NULL )
                        delete subsystems_template_reader;
                }
            }
            // Controllers are not required to have an input control space (!)
            if( parameters::has_attribute("input_control_space", reader, template_reader) )
            {
                if (input_control_space == NULL)
                {
                    PRX_FATAL_S ("Input control space is NULL, however input_control_space is specified in input!");
                }
                if( reader->has_attribute("input_control_space") )
                {
                    input_control_space->init(reader->get_child("input_control_space").get());
                }
                else if( template_reader != NULL )
                {
                    input_control_space->init(template_reader->get_child("input_control_space").get());
                }
            }

            if( parameters::has_attribute("state_space", reader, template_reader) )
            {
                if( controller_state_space != NULL )
                {
                    if( reader->has_attribute("state_space") )
                    {
                        controller_state_space->init(reader->get_child("state_space").get());
                    }
                    else if( template_reader != NULL )
                    {
                        controller_state_space->init(template_reader->get_child("state_space").get());
                    }
                }
                else
                {
                    PRX_FATAL_S("Attempted to load controller state space when it was NULL!");
                }
            }
            
            if ( parameters::has_attribute("sensing_info", reader, template_reader))
            {
                PRX_DEBUG_COLOR ("Controller : " << pathname << " has sensing info defined.", PRX_TEXT_MAGENTA);
   
                sensing_info_t* new_info = parameters::create_from_loader<sensing_info_t > ("prx_simulation", reader, "sensing_info", template_reader, "sensing_info");
                new_info->link_active(&active);
                PRX_DEBUG_COLOR("BEFORE initializing sensing info...", PRX_TEXT_RED);
                parameters::initialize(new_info, reader, "sensing_info", template_reader, "sensing_info");
                PRX_DEBUG_COLOR("AFTER initializing sensing info...", PRX_TEXT_RED);

                sensing_info.push_back(new_info);

            }
            
            this->construct_spaces();
        }

        void controller_t::add_system(const std::string& path, system_ptr_t system)
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            PRX_DEBUG_COLOR("+++ ADD system : name : " << name << "  subpath: " << subpath, PRX_TEXT_GREEN);
            if( name.empty() )
            {
                throw invalid_path_exception("Name is empty");
            }
            else if( subpath.empty() )
            {
                //                PRX_DEBUG_S("pathname : " << pathname);
                system->set_pathname(pathname + "/" + name);
                //                PRX_DEBUG_S("subsystem path : " << pathname + "/" + name);
                subsystems[name] = system;
                //                PRX_DEBUG_S("set subsytem : " << name << "  under path : " << pathname + "/" + name);
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

        void controller_t::remove_system(const std::string& path)
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            PRX_DEBUG_COLOR("--- REMOVE system : name : " << name << "  subpath : " << subpath, PRX_TEXT_CYAN);
            if( subsystems.find(name) != subsystems.end() )
                if( subpath.empty() )
                {
                    subsystems.erase(name);
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

            if( subsystems.size() == 0 )
                throw removal_exception();
        }

        void controller_t::replace_system(const std::string& path, system_ptr_t system)
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            if( subsystems.find(name) != subsystems.end() )
                if( subpath.empty() )
                    subsystems[name] = system;
                else
                    subsystems[name]->replace_system(subpath, system);
            else
                throw invalid_path_exception("The system " + name + " does not exist in the path " + pathname);

            construct_spaces();
            verify();
        }

        system_ptr_t controller_t::get_system(const std::string& path) const
        {
            std::string subsystem_name, subsystem_path;
            boost::tie(subsystem_name, subsystem_path) = split_path(path);

            if( subsystems.find(subsystem_name) != subsystems.end() &&
                std::count(path.begin(), path.end(), '/') == 0 )
            {
                return subsystems[subsystem_name];
            }
            else if( subsystems.find(subsystem_name) != subsystems.end() )
            {
                return subsystems[subsystem_name]->get_system(subsystem_path);
            }
            else
            {
                PRX_ERROR_S("Invalid path for get system" << path);
                throw invalid_path_exception(path);
            }
        }

        void controller_t::propagate(const double simulation_step)
        {
            if( active )
            {
                for( subsystems_iter = subsystems.begin(); subsystems_iter != subsystems.end(); ++subsystems_iter )
                {
                    if( subsystems_iter->second->is_active() )
                    {
                        subsystems_iter->second->propagate(simulation_step);
                    }
                }
            }
        }

        void controller_t::compute_control()
        {
            if( active )
            {
                for( subsystems_iter = subsystems.begin(); subsystems_iter != subsystems.end(); ++subsystems_iter )
                {
                    if( subsystems_iter->second->is_active() )
                    {
                        subsystems_iter->second->compute_control();
                    }
                }
            }
        }

        const space_t* controller_t::get_state_space() const
        {
            return state_space;
        }

        const space_t* controller_t::get_control_space() const
        {
            return input_control_space;
        }

        void controller_t::set_param(const std::string& path, const std::string& parameter_name, const boost::any& value)
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            if( name.empty() )
                set_param(parameter_name, value);
            else if( subsystems.find(name) != subsystems.end() )
                subsystems[name]->set_param(subpath, parameter_name, value);
            else
                throw invalid_path_exception(path);
        }

        void controller_t::update_phys_configs(config_list_t &configs, unsigned& index) const
        {
            for( hash_t<std::string, system_ptr_t >::const_iterator subsystems_const_iter = subsystems.begin(); subsystems_const_iter != subsystems.end(); ++subsystems_const_iter )
            {
                if( subsystems_const_iter->second->is_active() || simulation::update_all_configs )
                    subsystems_const_iter->second->update_phys_configs(configs, index);
            }
        }

        void controller_t::update_phys_geoms(geom_map_t &geoms) const
        {
            for( hash_t<std::string, system_ptr_t >::const_iterator subsystems_const_iter = subsystems.begin(); subsystems_const_iter != subsystems.end(); ++subsystems_const_iter )
                subsystems_const_iter->second->update_phys_geoms(geoms);
        }
        
        void controller_t::get_sensed_geoms(geom_map_t &geoms) const
        {
            for( hash_t<std::string, system_ptr_t >::const_iterator subsystems_const_iter = subsystems.begin(); subsystems_const_iter != subsystems.end(); ++subsystems_const_iter )
                subsystems_const_iter->second->get_sensed_geoms(geoms);
        }


        void controller_t::verify() const
        {
            // Check our spaces to make sure they exist
            //    PRX_ASSERT(input_control_space != NULL);
            PRX_ASSERT(output_control_space != NULL);
            PRX_ASSERT(state_space != NULL);

            // Controllers are not allowed to be leaves in the system tree
            PRX_ASSERT(!subsystems.empty());

            // Verify validity of our spaces
            if( input_control_space != NULL )
                input_control_space->verify();
            output_control_space->verify();
            state_space->verify();
            if( controller_state_space )
                controller_state_space->verify();

            hash_t<std::string, system_ptr_t >::const_iterator subsystems_const_iter;

            // Call our subsystems verify
            for( subsystems_const_iter = subsystems.begin(); subsystems_const_iter != subsystems.end(); ++subsystems_const_iter )
            {
                subsystems_const_iter->second->verify();
            }

            // Check to make sure our output_control_space == subsytems->get_control_space()
            std::string space_name;
            const space_t* cspace;
            //    subsystems_const_iter = subsystems.begin();
            //    cspace = subsystems_const_iter->second->get_control_space();
            //    if (cspace != NULL)
            //        space_name = cspace->get_space_name();
            for( unsigned i = 0; i < this->subsystem_names.size(); i++ )
            {
                cspace = subsystems[subsystem_names[i]]->get_control_space();
                if( cspace != NULL && cspace->get_space_name() != "EMPTY" )
                {
                    if( !space_name.empty() )
                        space_name += "|";
                    space_name += cspace->get_space_name();
                    //                PRX_ERROR_S ("Space name: " << space_name);
                }
            }
            //    if (!space_name.empty())
            //        space_name.erase(space_name.size());

            // Throw an exception if we do not match control spaces
            if( std::strcmp(space_name.c_str(), output_control_space->get_space_name().c_str()) )
            {
                PRX_ERROR_S("Output control space: " << output_control_space->get_space_name() << " does not match subsystem's input control space: " << space_name.c_str());
                throw space_mismatch_exception();
            }
        }

        system_ptr_t controller_t::create_subsystem(const std::string& path, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            system_ptr_t subsystem;

            subsystem.reset(parameters::create_from_loader<system_t > ("prx_simulation", reader, "", template_reader, ""));
            subsystem->set_pathname(path);
            subsystem->init(reader, template_reader);

            return subsystem;
        }

        void controller_t::construct_spaces()
        {
            if( output_control_space )
            {
                if( computed_control )
                    output_control_space->free_point(computed_control);
                delete output_control_space;
            }
            if( state_space )
                delete state_space;

            std::vector<const space_t*> spaces;
            for( unsigned i = 0; i < subsystem_names.size(); i++ )
            {
                spaces.push_back(subsystems[subsystem_names[i]]->get_control_space());
            }
            output_control_space = new space_t(spaces);
            computed_control = output_control_space->alloc_point();

            spaces.clear();
            for( unsigned i = 0; i < subsystem_names.size(); i++ )
            {
                spaces.push_back(subsystems[subsystem_names[i]]->get_state_space());
            }
            state_space = new space_t(spaces, controller_state_space);

            if( state_memory.empty() && controller_state_space != NULL )
            {
                state_memory.resize(controller_state_space->get_dimension());

                foreach(double* mem, state_memory)
                {
                    mem = new double;
                }
            }
        }

        void controller_t::update_vis_info() const
        {
            for( hash_t<std::string, system_ptr_t >::const_iterator subsystems_const_iter = subsystems.begin(); subsystems_const_iter != subsystems.end(); ++subsystems_const_iter )
                subsystems_const_iter->second->update_visualization_info();
        }

        void controller_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            system_t::set_param(parameter_name, value);
        }

        system_graph_t::directed_vertex_t controller_t::update_system_graph(system_graph_t& graph)
        {
            system_graph_t::directed_vertex_t v = graph.add_node(pathname, false);

            for( unsigned i = 0; i < subsystem_names.size(); i++ )
            {
                system_graph_t::directed_vertex_t new_v = subsystems[subsystem_names[i]]->update_system_graph(graph);
                graph.add_edge(v, new_v, subsystem_names[i], subsystems[subsystem_names[i]]);
            }
            return v;
        }

        void controller_t::set_active(bool in_active, const std::string& path)
        {
            //    PRX_DEBUG_S ("Set active: " << in_active);
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            if( path.empty() )
            {
                // PRX_DEBUG_S("Found system to set active: " << pathname);
                active = in_active;
            }
            else if( subsystems.find(name) != subsystems.end() )
            {
                // PRX_DEBUG_S("ACTIVE_FLAG: " << in_active << " " << subpath);
                subsystems[name]->set_active(in_active, subpath);
            }
            else
                throw invalid_path_exception("The system " + name + " does not exist in the path " + pathname);
        }

        space_t* controller_t::get_controller_state_space() const
        {
            return controller_state_space;
        }
        
        std::vector<sensing_info_t*> controller_t::get_sensing_info()
        {
            std::vector<sensing_info_t*> returned_sensing_info;
            returned_sensing_info.insert(returned_sensing_info.begin(), sensing_info.begin(), sensing_info.end());
            foreach(std::string name, subsystem_names)
            {
                // If subsystem is not a plant
                controller_t* controller = dynamic_cast<controller_t*>(subsystems[name].get());
                
                if ( controller != NULL )//&& controller->is_active())
                {
                    std::vector<sensing_info_t*> subtree_sensing_info;
                    subtree_sensing_info = controller->get_sensing_info();
                    returned_sensing_info.insert(returned_sensing_info.end(), subtree_sensing_info.begin(), subtree_sensing_info.end());
                }
            }
            
            return returned_sensing_info;
        }

        void controller_t::append_contingency(plan_t& result_plan, double duration)
        {

            foreach(std::string name, subsystem_names)
            {
                subsystems[name]->append_contingency(result_plan, duration);
            }
        }
    }
}


