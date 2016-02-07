/**
 * @file application.cpp
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

#include "prx/simulation/applications/application.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/simulation/systems/obstacle.hpp"

#include "prx/simulation/communication/sim_base_communication.hpp"
#include "prx/simulation/communication/planning_comm.hpp"
#include "prx/simulation/communication/simulation_comm.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"
#include "prx/utilities/communication/tf_broadcaster.hpp"

#include <boost/range/adaptor/map.hpp> //adaptors
#include <fstream>
#include "prx/utilities/definitions/sys_clock.hpp"

namespace prx
{
    using namespace util;

    namespace sim
    {
        using namespace comm;

        pluginlib::ClassLoader<application_t> application_t::loader("prx_simulation", "prx::sim::application_t");

        //sys_clock_t timer_thing;
        //double average_framerate;
        //double frame_total;

        application_t::application_t()
        {
            tf_broadcaster = NULL;
            simulator_running = false;
            simulator_counter = 0;
            simulator_mode = 0;
            loop_counter = 0;
            loop_total = 0.0;
            new_geometries_info = false;
            //    average_framerate = 0;
            //    frame_total = 0;

            selected_point.resize(3);
            selected_path.clear();
            collision_list = new vector_collision_list_t();

            stores_states = replays_states = false;


        }

        application_t::~application_t()
        {
            delete tf_broadcaster;
            delete plan_comm;
            delete sim_comm;
            delete vis_comm;
        }

        void application_t::init(const parameter_reader_t * const reader)
        {
            simulation::update_all_configs = true;
            std::string plugin_type_name;
            plugin_type_name = reader->get_attribute("sim_to_plan_comm", "planning_comm");
            plan_comm = sim_base_communication_t::get_loader().createUnmanagedInstance("prx_simulation/" + plugin_type_name);
            plan_comm->link_application(this);

            plugin_type_name = reader->get_attribute("sim_comm", "simulation_comm");
            sim_comm = sim_base_communication_t::get_loader().createUnmanagedInstance("prx_simulation/" + plugin_type_name);
            sim_comm->link_application(this);

            plugin_type_name = reader->get_attribute("sim_to_vis_comm", "visualization_comm");
            vis_comm = sim_base_communication_t::get_loader().createUnmanagedInstance("prx_simulation/" + plugin_type_name);
            vis_comm->link_application(this);


            PRX_DEBUG_COLOR("Initializing the application", PRX_TEXT_GREEN);
            simulator = reader->initialize_from_loader<simulator_t > ("simulator", "prx_simulation");
            
            visualize = reader->get_attribute_as<bool>("application/visualize", true);
            screenshot_every_simstep = reader->get_attribute_as<bool>("application/screenshot_every_simstep", false);

            simulation_control_space = simulator->get_control_space();
            simulation_state_space = simulator->get_state_space();
            simulation_state = simulation_state_space->alloc_point();
            simulation_control = simulation_control_space->alloc_point();
            
            

            deserialize_sim_file = reader->get_attribute_as<std::string > ("application/deserialize_sim_file", "");
            if( !deserialize_sim_file.empty() )
            {
                this->replays_states = true;
                replay_simulation_states.link_space(simulation_state_space);
                deserialize_simulation_states(deserialize_sim_file, replay_simulation_states);
            }

            serialize_sim_file = reader->get_attribute_as<std::string > ("application/serialize_sim_file", "");
            if( !serialize_sim_file.empty() )
            {
                store_simulation_states.link_space(simulation_state_space);
                this->stores_states = true;
            }



            PRX_ASSERT(simulator != NULL);
            tf_broadcaster = new tf_broadcaster_t;

            sim_key_name = "prx/input_keys/" + int_to_str(PRX_KEY_RETURN);
            if( !ros::param::has(sim_key_name) )
                ros::param::set(sim_key_name, false);

            simulator->update_system_graph(sys_graph);
            sys_graph.get_plant_paths(plant_paths);
            if( visualize )
            {
                PRX_PRINT ("Visualizing plants!", PRX_TEXT_CYAN);
                vis_comm->send_plants(ros::this_node::getName(), plant_paths);
                vis_comm->visualize_obstacles(ros::this_node::getName().substr(1,ros::this_node::getName().length())+"/"+ simulator->get_pathname());
            }

            
            init_collision_list(reader->get_child("application").get());

            hash_t<std::string, int> comm_systems;

            std::vector<plant_t*> get_plants;
            //SGC: only need plant pointers here
            sys_graph.get_plants(get_plants);

            foreach(plant_t* sys, get_plants)
            {
                PRX_DEBUG_COLOR("Plant: " << sys->get_pathname(), PRX_TEXT_GREEN);
                if( !sys->is_active() && visualize )
                {
                    vis_comm->visualize_plant(sys->get_pathname(), PRX_VIS_TEMPORARY_REMOVE);
                }
            }
            
            if (simulator->get_sensing_model() != NULL)
            {
                PRX_WARN_S ("Sensing model not null, proceeding with initialization of sensors.");
                initialize_sensing();
            }
            else
            {
                PRX_ERROR_S ("Sensing model is null, cannot initialize sensors.");
            }
            vis_comm->publish_markers();
            
        }
        
        void application_t::initialize_sensing()
        {            
            PRX_ERROR_S("Initialize sensing!");
            simulator->initialize_sensing();
        }

        bool application_t::running()
        {

            return true;
        }

        void application_t::frame(const ros::TimerEvent& event)
        {
            //    frame_total++;
            //    double timed = timer_thing.measure();
            ////    if (timed > .051|| timed < 0.049)
            //    {
            //        average_framerate += timed;
            //        PRX_ERROR_S ("Average Frame rate: " << average_framerate/frame_total);
            //    }
            //    timer_thing.reset();

            handle_key();

            if( simulator_mode == 1 )
            {
                if( simulator_counter > 0 )
                {
                    simulator_running = true;
                    simulator_counter--;
                    loop_timer.reset();
                }
                else
                    simulator_running = false;
            }
            if( loop_timer.measure() > 1.0 )
                loop_timer.reset();
            if( simulator_running )
            {

                if( replays_states )
                {
                    if( loop_counter > (int)replay_simulation_states.size() )
                        loop_counter = 0;
                    simulation_state_space->copy_from_point(replay_simulation_states[loop_counter]);
                    simulation_state_space->copy_to_point(simulation_state);
                }
                else
                {
                    simulator->push_state(simulation_state);
                    simulator->push_control(simulation_control);
                    simulator->propagate_and_respond();
                    simulation_state_space->copy_to_point(simulation_state);
                }
                if( stores_states )
                    store_simulation_states.copy_onto_back(simulation_state);

                if( screenshot_every_simstep )
                {
                    ((visualization_comm_t*)vis_comm)->take_screenshot(0, 1);
                }
                loop_total += loop_timer.measure_reset();
                loop_counter++;
                loop_avg = loop_total / loop_counter;

            }
            tf_broadcasting();


            //    if ( ground_truth_timer.measure() > ground_truth_time_limit )
            //    {
            //        sim_comm->publish_system_states();
            //        ground_truth_timer.reset();
            //    }

        }

        void application_t::serialize_simulation_states(const std::string& filename)
        {
            // Writes the state of the simulation to a file
            std::printf(" Inside application simulation  serialization now, saving to file: %s", filename.c_str());
            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/prx_output/");
            std::string file = dir + filename;
            std::printf("File directory is: %s", file.c_str());
            store_simulation_states.save_to_file(file);
            PRX_DEBUG_COLOR("Saved simulation states: " << store_simulation_states.print(), PRX_TEXT_MAGENTA);
        }

        void application_t::deserialize_simulation_states(const std::string& filename, trajectory_t& stored_sim_states)
        {
            // Reads the state of the simulation to a file
            std::printf(" Inside application simulation deserialization now, reading from file: %s", filename.c_str());
            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            dir += ("/prx_input/");
            std::string file = dir + filename;
            std::printf("File directory is: %s", file.c_str());
            replay_simulation_states.read_from_file(file);
            PRX_DEBUG_COLOR("Read simulation states: " << replay_simulation_states.print(), PRX_TEXT_MAGENTA);
        }

        void application_t::shutdown()
        {
            std::printf("------Application_t shutdown-----------");
            if( stores_states )
            {
                serialize_simulation_states(serialize_sim_file);
            }
        }

        //        void application_t::tf_broadcasting(const ros::TimerEvent& event)
        //        {
        //            //    PRX_DEBUG_S("tf_broadcasting");
        //            PRX_ASSERT(tf_broadcaster != NULL);
        //            config_list_t config_map;
        //            simulator->update_phys_configs(config_map);
        //            for( config_list_t::iterator iter = config_map.begin(); iter != config_map.end(); iter++ )
        //            {
        //                //        iter->second.print();
        //                tf_broadcaster->queue_config(iter->second, iter->first);
        //            }
        //            if( visualize )
        //                simulator->update_visualization_info(); //config_map);
        //            //    for( config_list_t::iterator iter = config_map.begin(); iter != config_map.end(); iter++ )
        //            //    {
        //            //        //        iter->second.print();
        //            //        tf_broadcaster->queue_config(iter->second, iter->first);
        //            //    }
        //            // TODO: Remove obstacles from TF, read in config from parameter server
        //            simulator->update_obstacles_configs(config_map);
        //            for( config_list_t::iterator iter = config_map.begin(); iter != config_map.end(); iter++ )
        //            {
        //                //        PRX_DEBUG_S("The name is: " << iter->first);
        //                //        iter->second.print();
        //                tf_broadcaster->queue_config(iter->second, iter->first);
        //            }
        //
        //            tf_broadcaster->broadcast_configs();
        //
        //        }

        void application_t::tf_broadcasting()
        {
            PRX_ASSERT(tf_broadcaster != NULL);
            config_list_t config_map;
            unsigned index = 0;
            simulator->update_phys_configs(config_map, index);

            for( config_list_t::iterator iter = config_map.begin(); iter != config_map.end(); iter++ )
            {
                //        iter->second.print();
                tf_broadcaster->queue_config(iter->second, iter->first);
            }
            if( visualize )
                simulator->update_visualization_info();
            tf_broadcaster->broadcast_configs();

        }

        void application_t::info_broadcasting(const ros::TimerEvent& event)
        {
            if( new_geometries_info )
            {

            }
        }

        void application_t::geom_broadcasting(const ros::TimerEvent& event) {
            //    geom_info_map_t geom_map;
            //    simulator->update_info_geoms(geom_map);
            //    if(geom_map.size() > 0)
            //    {
            //        PRX_ERROR_S ("Sending extra geometries!");
            //        comm->send_extra_geometries(geom_map);
            //    }
        }

        void application_t::set_running_simulator(bool running)
        {
            simulator_running = running;
        }

        void application_t::handle_key()
        {
            //    ros::param::getCached(sim_key_name, simulator_running);
            switch( key )
            {
                case PRX_KEY_TAB:
                    // Increments number of simulation steps to execute
                    simulator_mode = 1;
                    simulator_counter++;
                    // Consume key to avoid redundant frame steps
                    key = -1;
                    break;
                case PRX_KEY_RETURN:
                    // Toggle simulator running
                    simulator_mode = 2;
                    simulator_running = !simulator_running;
                    if( simulator_running )
                    {
                        PRX_INFO_S("Simulator is running!");
                    }
                    else
                    {
                        PRX_INFO_S("Simulator is stopped");
                    }
                    key = -1;
                    break;
                default:
                    break;
            }

            //    ros::param::set("input_key", key);
        }

        void application_t::set_pressed_key(int inkey)
        {
            key = inkey;
        }

        void application_t::set_selected_point(double x, double y, double z)
        {
            selected_point.set(x, y, z);
        }

        void application_t::set_camera(const vector_t& new_center, const vector_t& new_eye)
        {
            center = new_center;
            eye = new_eye;
        }

        void application_t::set_sim_state(const std::vector<double>& new_sim_state)
        {
            simulation_state_space->set_from_vector(new_sim_state, simulation_state);
            simulator->push_state(simulation_state);
        }

        simulator_t* application_t::get_simulator()
        {
            return simulator;
        }

        void application_t::init_collision_list(const parameter_reader_t * const reader)
        {
            std::string plant_name;

            vector_collision_list_t* black_list = new vector_collision_list_t();
            hash_t<std::string, system_ptr_t> obstacles = simulator->get_obstacles();

            foreach(std::string obst, obstacles | boost::adaptors::map_keys)
            {
                PRX_DEBUG_COLOR("obstacle : " << obst, PRX_TEXT_GREEN);
            }

            hash_t<std::string, plant_t*> plants;
            sys_graph.get_path_plant_hash(plants);

            foreach(std::string name1, plants | boost::adaptors::map_keys)
            {
                PRX_DEBUG_COLOR("plant: " << name1, PRX_TEXT_GREEN);
            }

            int index = 0;

            if( reader->has_attribute("black_list") )
            {

                foreach(const parameter_reader_t* list_reader, reader->get_list("black_list"))
                {
                    index++;
                    std::vector<const parameter_reader_t*> r = list_reader->get_list("");

                    if( r.size() != 2 )
                        PRX_FATAL_S("The pair " << index << " in black list is wrong. Has to be a systems with a list of systems.");

                    plant_name = r[0]->get_attribute("");

                    foreach(const parameter_reader_t* systems_reader, r[1]->get_list(""))
                    {
                        black_list->add_pair(plant_name, systems_reader->get_attribute(""));
                    }
                }
            }

            if( reader->has_attribute("white_list") )
            {
                index = 0;

                foreach(const parameter_reader_t* list_reader, reader->get_list("white_list"))
                {
                    index++;
                    std::vector<const parameter_reader_t*> r = list_reader->get_list("");

                    if( r.size() != 2 )
                        PRX_FATAL_S("The pair " << index << " in white list is wrong. Has to be a systems with a list of systems.");

                    plant_name = r[0]->get_attribute("");

                    foreach(const parameter_reader_t* systems_reader, r[1]->get_list(""))
                    {
                        std::string name2 = systems_reader->get_attribute("");
                        if( plants[name2] != NULL )
                            add_system_system_in_collision_list(plant_name, name2, black_list, plants);
                        else if( obstacles["simulator/obstacles/" + name2] != NULL )
                            add_system_obstacle_in_collision_list(plant_name, "simulator/obstacles/" + name2, black_list, plants, obstacles);
                        else
                            PRX_ERROR_S("The system/obstacles " << name2 << " does not exist in the simulation.");
                    }
                }
            }
            else
            {
                collision_list_all_vs_all(plants, obstacles, black_list);
            }

            //Add self-collisions
            if( reader->has_attribute("self_collision") )
            {

                foreach(const parameter_reader_t* r, reader->get_list("self_collision"))
                {
                    plant_name = r->get_attribute("");
                    add_system_self_collision_list(plant_name, black_list, plants);
                }
            }

            simulator->link_collision_list(collision_list);
            delete black_list;
        }

        void application_t::add_system_self_collision_list(const std::string& system, const vector_collision_list_t* black_list, hash_t<std::string, plant_t*>& plants)
        {
            std::vector<std::string>* geom_names = plants[system]->get_geometries_names();
            for( unsigned i = 0; i < geom_names->size() - 1; ++i )
                for( unsigned j = i + 1; j < geom_names->size(); ++j )
                {
                    collision_list->add_pair((*geom_names)[i], (*geom_names)[j]);
                }
        }

        void application_t::add_system_system_in_collision_list(const std::string& system1, const std::string& system2, const vector_collision_list_t* black_list, hash_t<std::string, plant_t*>& plants)
        {
            if( system1 != system2 && !black_list->pair_in_list(system1, system2) )
            {
                std::vector<std::string>* geom_names1 = plants[system1]->get_geometries_names();
                std::vector<std::string>* geom_names2 = plants[system2]->get_geometries_names();

                foreach(std::string n1, *geom_names1)
                {

                    foreach(std::string n2, *geom_names2)
                    {
                        //                        PRX_INFO_S("adding the pair : " << n1 << " -> " << n2);
                        collision_list->add_pair(n1, n2);
                    }
                }

                plants[system1]->update_collision_list(collision_list);
                plants[system2]->update_collision_list(collision_list);
            }
        }

        void application_t::add_system_obstacle_in_collision_list(const std::string& system, const std::string& obstacle, const vector_collision_list_t* black_list, hash_t<std::string, plant_t*>& plants, hash_t<std::string, system_ptr_t>& obstacles)
        {
            if( !black_list->pair_in_list(system, obstacle) )
            {
                std::vector<std::string>* geom_names1 = plants[system]->get_geometries_names();
                std::vector<std::string>* geom_names2 = static_cast<obstacle_t*>(obstacles[obstacle].get())->get_geometries_names();

                foreach(std::string n1, *geom_names1)
                {

                    foreach(std::string n2, *geom_names2)
                    {
                        collision_list->add_pair(n1, n2);
                    }
                }
                plants[system]->update_collision_list(collision_list);
            }
        }

        void application_t::collision_list_all_vs_all(hash_t<std::string, plant_t*>& plants, hash_t<std::string, system_ptr_t>& obstacles, const vector_collision_list_t* black_list)
        {

            // Optimized version
            for( hash_t<std::string, plant_t*>::const_iterator it = plants.begin(); it != plants.end(); it++ )
            {
                hash_t<std::string, plant_t*>::const_iterator temp_it = it;
                temp_it++;
                for( hash_t<std::string, plant_t*>::const_iterator it2 = temp_it; it2 != plants.end(); it2++ )
                {
                    //                    PRX_INFO_S(it->first << " - " << it2->first);
                    add_system_system_in_collision_list(it->first, it2->first, black_list, plants);
                }

                foreach(std::string obst, obstacles | boost::adaptors::map_keys)
                {
                    add_system_obstacle_in_collision_list(it->first, obst, black_list, plants, obstacles);
                }
            }
            // Old TDK method
            //    foreach(std::string name1, plants | boost::adaptors::map_keys)
            //    {
            //        foreach(std::string name2, plants | boost::adaptors::map_keys)
            //            add_system_system_in_collision_list(name1, name2, black_list, plants);
            //        foreach(std::string obst, obstacles | boost::adaptors::map_keys)
            //            add_system_obstacle_in_collision_list(name1, obst, black_list, plants, obstacles);
            //    }

        }

        bool application_t::is_visualizing()
        {
            return visualize;
        }

        pluginlib::ClassLoader<application_t>& application_t::get_loader()
        {
            return loader;
        }

    }
}

