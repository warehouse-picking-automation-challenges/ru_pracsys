/**
 * @file simulator.hpp 
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

#include "prx/simulation/simulators/simulator.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/simulation/system_graph.hpp"

#include <boost/range/adaptor/map.hpp> //adaptors
#include <boost/tuple/tuple.hpp> // boost::tie
#include <pluginlib/class_loader.h>

namespace prx
{
    using namespace util;

    namespace sim
    {

        pluginlib::ClassLoader<simulator_t> simulator_t::loader("prx_simulation", "prx::sim::simulator_t");

        namespace simulation
        {
            double simulation_step;
            bool update_all_configs;
        }

        simulator_t::simulator_t()
        {
            pathname = "simulator";
            collision_checker = NULL;
            simulation::simulation_step = 0.0;
            sensing = NULL;

        }

        simulator_t::~simulator_t()
        {
            if (sensing != NULL)
                delete sensing;
            delete collision_checker;
            obstacles.clear();
        }

        void simulator_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            geom_map_t geom_map;
            std::string obstacle_name;
            std::string template_name;

            simulation::simulation_step = parameters::get_attribute_as<double>("simulation_step", reader, template_reader);

            if( parameters::has_attribute("pathname", reader, template_reader) )
            {
                pathname = parameters::get_attribute("pathname", reader, template_reader);
                PRX_DEBUG_COLOR("Set pathname: " << pathname, PRX_TEXT_GREEN);
            }

            if( reader->has_attribute("collision_checker") )
            {
                PRX_DEBUG_COLOR("Collision checker attribute detected", PRX_TEXT_GREEN);
                collision_checker = reader->initialize_from_loader<collision_checker_t > ("collision_checker", "prx_simulation");
            }
            else if( template_reader != NULL )
            {
                collision_checker = template_reader->initialize_from_loader<collision_checker_t > ("collision_checker", "prx_simulation");
            }
            else
            {
                PRX_FATAL_S("Simulator does not have collision checker!");
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

            router_t::init(reader, template_reader);
            
//            if( parameters::has_attribute("subsystems", reader, template_reader) )
//            {
//                parameter_reader_t::reader_map_t subsystem_map = parameters::get_map("subsystems", reader, template_reader);
//
//                foreach(const parameter_reader_t::reader_map_t::value_type key_value, subsystem_map)
//                {
//                    PRX_DEBUG_COLOR("Composite controller subsystem " << key_value.first, PRX_TEXT_GREEN);
//                    const parameter_reader_t* subsystems_template_reader = NULL;
//
//                    if( key_value.second->has_attribute("template") )
//                    {
//                        template_name = key_value.second->get_attribute("template");
//                        PRX_DEBUG_COLOR("We have a template!:" << template_name, PRX_TEXT_CYAN);
//                        // TODO: Find a way to load templates under namespaces more cleanly.
//                        subsystems_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);
//
//                    }
//                    subsystems[key_value.first] = create_subsystem(pathname + "/" + key_value.first, key_value.second, subsystems_template_reader);
//                    subsystem_names.push_back(key_value.first);
//                    PRX_DEBUG_COLOR("set subsystem : " << key_value.first << "  under path : " << pathname + "/" + key_value.first, PRX_TEXT_GREEN);
//                    if( subsystems_template_reader != NULL )
//                        delete subsystems_template_reader;
//                }
//            }
//            construct_spaces();

            simulator_state = state_space->alloc_point();

            update_phys_geoms(geom_map);
            unsigned index = 0;
            update_phys_configs(config_list, index);

            system_graph_t::directed_vertex_t v = update_system_graph(sys_graph);
            sys_graph.system_graph[v].name = "simulator";
            //    sys_graph.system_graph[v].system.reset(this);
            sys_graph.get_path_plant_hash(plants);

            foreach(plant_t* plant, plants | boost::adaptors::map_values)
            {
                //find the place in the structure where this plant's state space exists
                std::pair<unsigned, unsigned> space_bounds = state_space->get_subspace(plant->get_state_space());
                if( space_bounds.first != space_bounds.second )
                {
                    system_intervals[plant] = space_bounds;
                }
            }

            collision_checker->add_bodies(geom_map, plants);
            collision_checker->set_configurations(config_list);

            verify();

            /** Check for sensing */
            if (parameters::has_attribute("sensing", reader, template_reader))
            {
                PRX_DEBUG_COLOR("Simulator has sensing, proceeding with initialization.", PRX_TEXT_GREEN);
                sensing = parameters::initialize_from_loader<sensing_model_t>("prx_simulation", reader, "sensing",  template_reader, "sensing");
            }
            else
            {
                PRX_WARN_S("No sensing provided to simulator.  No sensors can be used.");
            }
        }

        bool simulator_t::in_collision()
        {
            unsigned index = 0;
            update_phys_configs(config_list, index);

            if( index < config_list.size() )
                config_list.resize(index);

            return collision_checker->in_collision(config_list);
        }

        double simulator_t::collision_distance()
        {
            unsigned index = 0;
            update_phys_configs(config_list, index);

            if( index < config_list.size() )
                config_list.resize(index);   
                
            return collision_checker->get_clearance(config_list);         
        }

        bool simulator_t::near_collision( double eps )
        {
            unsigned index = 0;
            update_phys_configs(config_list, index);
            
            if( index < config_list.size() )
                config_list.resize(index);
            
            return collision_checker->near_collision( eps, config_list );
        }

        void simulator_t::link_collision_list(collision_list_t* collision_list)
        {
            collision_checker->link_collision_list(collision_list);
        }

        collision_list_t* simulator_t::get_colliding_bodies()
        {
            unsigned index = 0;
            update_phys_configs(config_list, index);
            if( index < config_list.size() )
                config_list.resize(index);
            return collision_checker->colliding_bodies(config_list);
        }

        collision_list_t* simulator_t::get_near_colliding_bodies( double eps )
        {
            unsigned index = 0;
            update_phys_configs(config_list, index);
            if( index < config_list.size() )
                config_list.resize(index);
            return collision_checker->near_colliding_bodies(config_list, eps);
        }
        
        void simulator_t::add_system(const std::string& path, system_ptr_t system)
        {
            controller_t::add_system(path, system);

            geom_map_t map;
            system->update_phys_geoms(map);

            foreach(std::string name, map | boost::adaptors::map_keys)
            {
                collision_checker->add_body(name, map[name], (plant_t*)system.get());
            }

            map.clear();
        }

        void simulator_t::remove_system(const std::string& path)
        {
            geom_map_t map;
            system_ptr_t system = get_system("simulator/" + path);
            system->update_phys_geoms(map);

            foreach(std::string name, map | boost::adaptors::map_keys)
            {
                //        PRX_WARN_S("in map: " << name);
                collision_checker->remove_body(name);
            }

            map.clear();

            try
            {
                controller_t::remove_system(path);
            }
            catch( removal_exception e )
            {
                PRX_WARN_S("The simulator is without systems ");
            }

        }

        void simulator_t::replace_system(const std::string& path, system_ptr_t system)
        {
            controller_t::replace_system(path, system);
        }

        void simulator_t::update_obstacles_configs(config_list_t& map, unsigned& index)
        {
            foreach(std::string name, obstacles | boost::adaptors::map_keys)
            obstacles[name]->update_phys_configs(map, index);
        }

        void simulator_t::add_obstacle(const std::string& name, system_ptr_t obstacle)
        {
            geom_map_t geom_map;
            config_list_t config_map;
            unsigned index = 0;
            if( obstacles[name] == NULL )
            {
                PRX_DEBUG_COLOR("new obstacle : " << name, PRX_TEXT_GREEN);
                obstacles[name] = obstacle;
                obstacles[name]->update_phys_geoms(geom_map);
                obstacles[name]->update_phys_configs(config_map, index);
                foreach(std::string name, geom_map | boost::adaptors::map_keys)
                collision_checker->add_body(name, geom_map[name], obstacle.get(), true);
                collision_checker->set_configurations(config_map);
            }
        }

        void simulator_t::remove_obstacle(const std::string& name)
        {
            if( obstacles[name] != NULL )
            {
                obstacles.erase(name);
                collision_checker->remove_body(name);
            }
        }

        hash_t<std::string, system_ptr_t>& simulator_t::get_obstacles()
        {
            return obstacles;
        }

        void simulator_t::verify() const
        {
            if( collision_checker == NULL )
                throw invalid_system_exception("The simulator " + pathname + " does not have a collision checker");

            controller_t::verify();
        }

        void simulator_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            if( parameter_name == "collision_checker" )
            {
                if( (collision_checker = boost::any_cast< collision_checker_t* >(value)) == NULL )
                    throw invalid_element_exception("Collision_checker is not of type collision_checker_t");
            }
            else
            {
                router_t::set_param(parameter_name, value);
            }
        }

        system_ptr_t simulator_t::get_system(const std::string& system_path) const
        {

            std::string simulator_name, subsystem_path;
            boost::tie(simulator_name, subsystem_path) = split_path(system_path);
            return controller_t::get_system(subsystem_path);
        }

        void simulator_t::push_state(const state_t * const state)
        {
            state_space->copy_from_point(state);
            if (sensing != NULL)
            {
                sensing->update_sensed_state(state);
                sensing->sense(simulation::simulation_step);
            }
        }

        void simulator_t::push_control(const control_t * const control)
        {
            input_control_space->copy_from_point(control);
            router_t::compute_control();
        }

        void simulator_t::propagate(const double simulation_step)
        {
            for( subsystems_iter = subsystems.begin(); subsystems_iter != subsystems.end(); ++subsystems_iter )
            {
                if( subsystems_iter->second->is_active() )
                {
                    subsystems_iter->second->propagate(simulation_step);
                }
            }
        }

        void simulator_t::push_control()
        {
            for( subsystems_iter = subsystems.begin(); subsystems_iter != subsystems.end(); ++subsystems_iter )
            {
                subsystems_iter->second->compute_control();
            }
        }

        state_t* simulator_t::pull_state()
        {
            state_space->copy_to_point(simulator_state);
            return simulator_state;
        }
        
        void simulator_t::initialize_sensing()
        {
            PRX_DEBUG_COLOR("------------------INITIALIZE SENSING----------------", PRX_TEXT_LIGHTGRAY);
            PRX_ASSERT_MSG(sensing != NULL, "Sensing is NULL!");
            
            sensing->initialize_sensors(this);             
            sensing->set_sensing_info(get_sensing_info());

        }
        
        sensing_model_t* simulator_t::get_sensing_model()
        {
            return sensing;
        }
        
        collision_checker_t* simulator_t::get_collision_checker()
        {
            return collision_checker;
        }

        pluginlib::ClassLoader<simulator_t>& simulator_t::get_loader()
        {
            return loader;
        }
        
        void simulator_t::update_obstacles_in_collision_checker()
        {
            config_list_t map;
            unsigned index = 0;
            foreach(std::string name, obstacles | boost::adaptors::map_keys)
                obstacles[name]->update_phys_configs(map, index);
            collision_checker->set_configurations(map);
        }


        void simulator_t::update_obstacle_geoms(geom_map_t &geoms) const
        {
            foreach(std::string name, obstacles | boost::adaptors::map_keys)
                obstacles[name]->update_phys_geoms(geoms);
        }

    }
}


