/**
 * @file world_model.cpp
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

#include "prx/utilities/definitions/defs.hpp"
#include "simulation/plants/manipulator.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/spaces/embedded_space.hpp"
#include "prx/utilities/spaces/mapping_functions/full_mapping.hpp"
#include "prx/utilities/spaces/mapping_functions/hide_mapping.hpp"
#include "prx/utilities/spaces/mapping_functions/obstacle_mapping.hpp"
#include "prx/simulation/simulators/no_collision_simulator.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp> //adaptors
#include <ros/ros.h>


namespace prx
{
    using namespace util;
    using namespace sim;

    namespace util
    {
        class bounds_t;
    }

    namespace plan
    {

        //pluginlib::ClassLoader<world_model_t> world_model_t::loader("prx_planning","prx::plan::plan::world_model_t");

        void world_model_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            collision_list = new vector_collision_list_t();
            const parameter_reader_t* world_model_reader = reader;

            std::string sim_type = world_model_reader->get_attribute_as<std::string > ("simulator/type");
            use_optimized_propagate = world_model_reader->get_attribute_as<bool>("use_optimized_propagate", true);
            phys_based_sim = (sim_type == "ode_simulator") || (sim_type == "bullet_simulator");
            respond = world_model_reader->get_attribute_as<bool>("respond", false);

            simulator = world_model_reader->initialize_from_loader<simulator_t > ("simulator", "prx_simulation");

            simulation_step = simulation::simulation_step;

            full_state = full_control = NULL;

            const space_t* full_state_space = simulator->get_state_space();
            const space_t* full_control_space = simulator->get_control_space();

            std::vector<double*> null_memory;

            context_name = "full_space";
            contexts["full_space"].planning_state_space = const_cast<space_t*>(full_state_space);
            contexts["full_space"].planning_control_space = const_cast<space_t*>(full_control_space);
            contexts["full_space"].active_space = new space_t("EMPTY", null_memory);
            contexts["full_space"].inactive_space = new space_t("EMPTY", null_memory);
            contexts["full_space"].temp_state = full_state_space->alloc_point();
            selected_state_space = contexts["full_space"].planning_state_space;
            selected_control_space = contexts["full_space"].planning_control_space;

            if( full_state == NULL && full_control == NULL )
            {
                full_state = full_state_space->alloc_point();
                full_control = full_control_space->alloc_point();
            }

            system_graph_t::directed_vertex_t v = simulator->update_system_graph(sys_graph);
            //    PRX_ERROR_S ("Testing sys ptr of simulator " << sys_graph.system_graph[v].system);
            sys_graph.system_graph[v].name = "simulator";
            //    sys_graph.system_graph[v].system.reset(simulator);
            //SGC: This only needs to be get plant names
            sys_graph.get_path_plant_hash(plants);
            sys_graph.get_path_system_hash(systems);
            //    PRX_ERROR_S ("Testing simulator " << systems["simulator"]);

            std::vector<system_ptr_t> systems_list;
            sys_graph.get_systems(systems_list);

            foreach(std::string name, systems | boost::adaptors::map_keys)
            {
                contexts["full_space"].activation_list[name] = true;
            }

            std::vector<mapping_function_t*> state_mappings;
            std::vector<mapping_function_t*> control_mappings;

            //construct all the embedded spaces specified.

            if( world_model_reader->has_attribute("planning_contexts") )
            {
                parameter_reader_t::reader_map_t contexts_map = world_model_reader->get_map("planning_contexts");

                foreach(const parameter_reader_t::reader_map_t::value_type key_value, contexts_map)
                {
                    bool use_embedded_space = false;
                    if(key_value.second->has_attribute("use_embedded_space"))
                    {
                        use_embedded_space = key_value.second->get_attribute_as<bool>("use_embedded_space");
                    }
                    PRX_INFO_S("Creating a planning context: "<<key_value.first<<" Embedding?: "<<use_embedded_space);
                    state_mappings.clear();
                    control_mappings.clear();
                    //initialize the activation list for this space

                    foreach(std::string name, systems | boost::adaptors::map_keys)
                    {
                        contexts[key_value.first].activation_list[name] = true;
                    }
                    //key_value.first is the name of the space to create
                    //key_value.second the map that consists of the default mapping_function to create

                    std::vector<const space_t*> planning_spaces;
                    std::vector<const space_t*> planning_controls;
                    std::vector<const space_t*> active_spaces;
                    std::vector<const space_t*> inactive_spaces;
                    bool nontrivial_embedding = false;
                    foreach(system_ptr_t system_pointer, systems_list)
                    {
                        controller_t* system = dynamic_cast<controller_t*>(system_pointer.get());
                        router_t* router_check = dynamic_cast<router_t*>(system_pointer.get());
                        space_t* space = NULL;
                        bool do_work = false;
                        bool specialized_controls = false;
                        std::pair<unsigned, unsigned> control_interval = full_control_space->get_subspace(system_pointer.get()->get_control_space());

                        if( router_check == NULL )
                        {
                            //if a controller has its own state
                            if( system != NULL && (space = system->get_controller_state_space()) != NULL )
                            {
                                do_work = true;
                            }
                            //if the controller doesn't have its own state, but has controls that contribute
                            else if( system != NULL && control_interval.first != control_interval.second )
                            {
                                specialized_controls = true;
                            }
                            //if a plant, always consider it.
                            else if( system == NULL )
                            {
                                space = const_cast<space_t*>(system_pointer.get()->get_state_space());
                                do_work = true;
                            }
                        }

                        if( do_work )
                        {
                            std::string sys_pathname = system_pointer.get()->get_pathname();
                            std::string slash = "/";
                            std::string embed_path = "/planning_contexts/";
                            std::string init_path;
                            const parameter_reader_t* init_reader;

                            std::size_t found = 0;

                            while( (found = sys_pathname.find(slash, found)) != std::string::npos )
                            {
                                sys_pathname.replace(found, slash.length(), "/subsystems/");
                                found += 12;
                            }

                            //should have replaced the slashes with subsystems so that we can find the parameters

                            // if( ros::param::has(world_model_reader->trace() + sys_pathname + embed_path + key_value.first) )
                            // PRX_PRINT(world_model_reader->trace() + sys_pathname + embed_path + key_value.first,PRX_TEXT_BROWN);
                            if(world_model_reader->has_attribute(sys_pathname+embed_path+key_value.first))
                            {
                                init_path = sys_pathname + embed_path + key_value.first;
                                init_reader = world_model_reader;
                                mapping_function_t* map = world_model_reader->create_from_loader<mapping_function_t > (init_path, "prx_utilities");
                                state_mappings.push_back(map);
                            }
                            else
                            {
                                init_path = key_value.first;
                                init_reader = key_value.second;
                                init_path = "default";
                                mapping_function_t* map = key_value.second->create_from_loader<mapping_function_t > ("default", "prx_utilities");
                                state_mappings.push_back(map);
                            }
                            //we should have created the mapping function for this system. We will delete them all if no nontrival mappings are used.

                            //based on the type, we need to do other behavior
                            if( std::strcmp(state_mappings.back()->mapping_name.c_str(), "hide_mapping") == 0 )
                            {
                                inactive_spaces.push_back(space);
                                if( system == NULL )
                                    contexts[key_value.first].activation_list[system_pointer.get()->get_pathname()] = false;
                                state_mappings.back()->domain = space->get_dimension();
                                state_mappings.back()->preimage_interval = full_state_space->get_subspace(space);
                                init_reader->initialize((state_mappings.back()), init_path);

                                if( control_interval.first != control_interval.second )
                                {
                                    // construct the control space embedding
                                    // create a hide embedding
                                    mapping_function_t* control_map_function = new hide_mapping_t();
                                    // initialize the mapping_name, domain, preimage_interval
                                    control_map_function->domain = system_pointer.get()->get_control_space()->get_dimension();
                                    control_map_function->preimage_interval = control_interval;
                                    control_map_function->init_spaces();
                                    control_mappings.push_back(control_map_function);
                                }
                            }
                            else if( std::strcmp(state_mappings.back()->mapping_name.c_str(), "obstacle_mapping") == 0 )
                            {
                                active_spaces.push_back(space);
                                state_mappings.back()->domain = space->get_dimension();
                                state_mappings.back()->preimage_interval = full_state_space->get_subspace(space);
                                init_reader->initialize((state_mappings.back()), init_path);

                                if( control_interval.first != control_interval.second )
                                {
                                    // construct the control space embedding
                                    // create an obstacle embedding
                                    mapping_function_t* control_map_function = new obstacle_mapping_t();
                                    // initialize the mapping_name, domain, preimage_interval
                                    control_map_function->domain = system_pointer.get()->get_control_space()->get_dimension();
                                    control_map_function->preimage_interval = control_interval;
                                    control_map_function->init_spaces();
                                    control_mappings.push_back(control_map_function);
                                }
                            }
                            else if( std::strcmp(state_mappings.back()->mapping_name.c_str(), "full_mapping") == 0 )
                            {
                                planning_spaces.push_back(space);
                                state_mappings.back()->domain = (state_mappings.back()->range) = space->get_dimension();
                                state_mappings.back()->output_space_name = space->get_space_name();
                                state_mappings.back()->preimage_interval = full_state_space->get_subspace(space);
                                init_reader->initialize((state_mappings.back()), init_path);

                                std::vector<prx::bounds_t*> bounds = space->get_bounds();
                                std::vector<double*> scales = space->get_scales();
                                std::vector<double> inscales;

                                foreach(double* scale, scales)
                                {
                                    inscales.push_back(*scale);
                                }
                                state_mappings.back()->get_embedded_subspace()->set_bounds(bounds);
                                state_mappings.back()->get_embedded_subspace()->set_scales(inscales);


                                if( control_interval.first != control_interval.second )
                                {
                                    planning_controls.push_back(system_pointer.get()->get_control_space());
                                }

                            }
                            else
                            {
                                nontrivial_embedding = true;
                                state_mappings.back()->preimage_interval = full_state_space->get_subspace(space);
                                init_reader->initialize((state_mappings.back()), init_path);


                                if( control_interval.first != control_interval.second )
                                {
                                    planning_controls.push_back(system_pointer.get()->get_control_space());
                                }
                            }
                        }
                        else if( specialized_controls )
                        {
                            // construct the control space embedding
                            // create a hide embedding
                            planning_controls.push_back(system_pointer.get()->get_control_space());
                        }
                    }
                    if(nontrivial_embedding || use_embedded_space)
                    {
                        contexts[key_value.first].planning_state_space = new embedded_space_t(state_mappings, const_cast<space_t*>(full_state_space));
                        foreach(mapping_function_t* func, state_mappings)
                        {
                            func->init_spaces();
                        }
                    }
                    else
                    {
                        contexts[key_value.first].planning_state_space = new space_t(planning_spaces);
                    }

                    //remove duplicate control spaces, they are not necessary
                    std::sort( planning_controls.begin(), planning_controls.end() );
                    planning_controls.erase( std::unique( planning_controls.begin(), planning_controls.end() ), planning_controls.end() );

                    contexts[key_value.first].planning_control_space = new space_t(planning_controls);
                    contexts[key_value.first].active_space = new space_t(active_spaces);
                    contexts[key_value.first].inactive_space = new space_t(inactive_spaces);
                    contexts[key_value.first].temp_state = contexts[key_value.first].planning_state_space->alloc_point();
                }
            }

            if( dynamic_cast<no_collision_simulator_t*>(simulator) == NULL )
                init_collision_list(world_model_reader);

            if (simulator->get_sensing_model() != NULL)
            {
                simulator->initialize_sensing();
            }
        }

        space_t* world_model_t::get_state_space() const
        {
            return selected_state_space;
        }

        space_t* world_model_t::get_control_space() const
        {
            return selected_control_space;
        }
        space_t* world_model_t::get_active_space() const
        {
            return contexts[context_name].active_space;
        }
        space_t* world_model_t::get_inactive_space() const
        {
            return contexts[context_name].inactive_space;
        }

        const space_t* world_model_t::get_full_state_space() const
        {
            return simulator->get_state_space();
        }

        const space_t* world_model_t::get_full_control_space() const
        {
            return simulator->get_control_space();
        }

        state_t* world_model_t::pull_state()
        {
            return selected_state_space->alloc_point();
        }

        void world_model_t::push_state(const state_t * const source)
        {
            //Assuming two things
            // 1) Two embedded values don't map to the same full value
            // 2) If the values have the same name, they are equivalent
            //PRX_LOG_DEBUG("RIGHT NOW COPY STATE DOESNT USE A NOTION OF TIME: AlSO NO NAME CHANGES");
            selected_state_space->copy_from_point(source);
            if( simulator->internal_state_push() )
            {
                get_full_state_space()->copy_to_point(full_state);
                // PRX_DEBUG_COLOR("Push State  " << get_full_state_space()->print_point(full_state,5), PRX_TEXT_LIGHTGRAY);
                simulator->push_state(full_state);
            }
            return;
        }

        void world_model_t::push_control(const control_t* source)
        {
            //Assuming two things
            // 1) Two embedded values don't map to the same full value
            // 2) If the values have the same name, they are equivalent
            //PRX_LOG_DEBUG("RIGHT NOW COPY CONTROL DOESNT USE A NOTION OF TIME: AlSO NO NAME CHANGES");

            selected_control_space->copy_from_point(source);
            simulator->push_control();

            return;
        }

        sim::simulator_t* world_model_t::get_simulator() const
        {
            return simulator;
        }

        const std::vector<config_t> world_model_t::get_configs(const state_t * const state, std::vector<std::string> system_names)
        {
            std::vector<config_t> configs;
            configs.clear();
            this->push_state(state);
            config_map.clear();
            unsigned index = 0;
            simulator->update_phys_configs(config_map,index);
            foreach(std::string name, system_names)
            {
                foreach(config_list_element_t element, config_map)
                {
                    if( element.first == name )
                        configs.push_back(element.second);
                }
            }
            return configs;
        }

        void world_model_t::reset(const state_t* initial_state)
        {
            PRX_FATAL_S("Please use push_state instead of reset! They do the same thing and reset is misleading.");
            this->push_state(initial_state);
        }

        // void world_model_t::propagate(const state_t * const state, const control_t * const control,
        //                               const double time, trajectory_t& traj)
        // {
        //     state_t * initial = contexts[context_name].temp_state;

        //     //to handle the numerical instability, add .1 i.e. time=.8499999 and sim_step = .017 would result in 49 instead of 50
        //     int steps = (int)((time / simulation_step) + .1);
        //     traj.clear();
        //     traj.copy_onto_back(initial);
        //     int i = 0;
        //     if( steps > 0 )
        //     {
        //         propagate_once(initial, control, simulation_step, initial);
        //         traj.copy_onto_back(initial);
        //     }

        //     if( phys_based_sim || !use_optimized_propagate )
        //     {
        //         for( i = 1; i < steps; i++ )
        //         {
        //             propagate_once(initial, control, simulation_step, initial);
        //             traj.copy_onto_back(initial);
        //         }
        //     }
        //     else
        //     {
        //         for( i = 1; i < steps; i++ )
        //         {
        //             propagate_optimized(initial, control, simulation_step, initial);
        //             traj.copy_onto_back(initial);
        //         }
        //     }
        // }

        // void world_model_t::propagate(const state_t * const state, const control_t * const control,
        //                               const double time, state_t* result)
        // {
        //     selected_state_space->copy_point(result, state);

        //     //to handle the numerical instability, add .1 i.e. time=.8499999 and sim_step = .017 would result in 49 instead of 50
        //     int steps = (int)((time / simulation_step) + .1);

        //     int i = 0;
        //     if( steps > 0 )
        //     {
        //         propagate_once(result, control, simulation_step, result);
        //     }

        //     if( phys_based_sim || !use_optimized_propagate )
        //     {
        //         selected_state_space->copy_to_point(result);
        //         for( i = 1; i < steps; i++ )
        //         {
        //             propagate_once(result, control, simulation_step, result);
        //         }
        //     }
        //     else
        //     {
        //         for( i = 1; i < steps; i++ )
        //         {
        //             propagate_optimized(result, control, simulation_step, result);
        //         }
        //         selected_state_space->copy_to_point(result);
        //     }

        //     return;
        // }

        void world_model_t::propagate_once(const state_t* state, const control_t* control, const double time, state_t* result)
        {
            this->push_state(state);
            this->push_control(control);
            if (respond)
                simulator->propagate_and_respond();
            else
                simulator->propagate(time);
            selected_state_space->copy_to_point(result);
        }

        void world_model_t::propagate_optimized(const state_t* state, const control_t* control, const double time, state_t* result)
        {
            this->push_control(control);
            if (respond)
                simulator->propagate_and_respond();
            else
                simulator->propagate(time);
            selected_state_space->copy_to_point(result);
        }

        void world_model_t::propagate_plan(const state_t * const state, const plan_t& plan, trajectory_t& traj, bool collision_in_traj, bool print_death )
        {
            this->push_state(state);
            state_t* initial = contexts[context_name].temp_state;
            selected_state_space->copy_point(initial, state);
            traj.clear();
            traj.copy_onto_back(initial);

            bool first_step = true;
            foreach(plan_step_t step, plan)
            {
                int steps = (int)((step.duration / simulation_step) + .1);
                int i = 0;
                if( steps > 0 )
                {
                    if( first_step )
                    {
                        i = 1;
                        first_step = false;
                        propagate_once(initial, step.control, simulation_step, initial);
                        traj.copy_onto_back(initial);
                        if( phys_based_sim || collision_in_traj )
                        {
                            bool incol = traj.in_collision( ) || simulator->in_collision();
                            //if( print_death && incol && !traj.in_collision() )
                            //{
                            //    collision_list_t* list = simulator->get_colliding_bodies();
                            //    PRX_WARN_S( "--" );
                            //    foreach(collision_pair_t pair, list->get_body_pairs())
                            //    {
                            //        PRX_WARN_S( "TEST " << pair.first << " " << pair.second);
                            //    }  
                            //}
                            traj.set_collision( incol );
                        }
                    }
                    if( phys_based_sim || !use_optimized_propagate )
                    {
                        for( ; i < steps; i++ )
                        {
                            propagate_once(initial, step.control, simulation_step, initial);
                            traj.copy_onto_back(initial);
                            if( phys_based_sim || collision_in_traj )
                            {
                                bool incol = traj.in_collision( ) || simulator->in_collision();
                                //if( print_death && incol && !traj.in_collision() )
                                //{
                                //    collision_list_t* list = simulator->get_colliding_bodies();
                                //    PRX_WARN_S( "--" );
                                //    foreach(collision_pair_t pair, list->get_body_pairs())
                                //    {
                                //        PRX_WARN_S( "TEST " << pair.first << " " << pair.second);
                                //    }  
                                //}
                                traj.set_collision( incol );
                            }
                        }
                    }
                    else
                    {
                        for( ; i < steps; i++ )
                        {
                            propagate_optimized(initial, step.control, simulation_step, initial);
                            traj.copy_onto_back(initial);
                            if( collision_in_traj )
                            {
                                bool incol = traj.in_collision() || simulator->in_collision();
                                //if( print_death && incol && !traj.in_collision() )
                                //{
                                //    collision_list_t* list = simulator->get_colliding_bodies();
                                //    PRX_WARN_S( "--" );
                                //    foreach(collision_pair_t pair, list->get_body_pairs())
                                //    {
                                //        PRX_WARN_S( "TEST " << pair.first << " " << pair.second);
                                //    }  
                                //}
                                traj.set_collision( incol );
                            }
                        }
                    }
                }
            }
        }

        void world_model_t::propagate_plan(const state_t * const state, const plan_t& plan, state_t* result)
        {
            this->push_state(state);
            selected_state_space->copy_point(result, state);

            bool first_step = true;
            foreach(plan_step_t step, plan)
            {
                int steps = (unsigned int)((step.duration / simulation_step) + .1);
                int i = 0;
                if( steps > 0 )
                {
                    if( first_step )
                    {
                        first_step = false;
                        i=1;
                        propagate_once(result, step.control, simulation_step, result);
                    }
                    if( phys_based_sim || !use_optimized_propagate )
                    {
                        selected_state_space->copy_to_point(result);
                        for( ; i < steps; i++ )
                        {
                            propagate_once(result, step.control, simulation_step, result);
                        }
                    }
                    else
                    {
                        for( ; i < steps; i++ )
                        {
                            propagate_optimized(result, step.control, simulation_step, result);
                        }
                        selected_state_space->copy_to_point(result);
                    }
                }
            }

            return;
        }

        bool world_model_t::valid_state(const state_t* state)
        {
            selected_state_space->copy_from_point(state);
            // if(!simulator->get_state_space()->satisfies_bounds(state,false))
            // {
            //    return false;
            // }
            return !simulator->in_collision();
        }

        bool world_model_t::near_valid_state( double eps, const state_t* state )
        {
            selected_state_space->copy_from_point( state );
            return !simulator->near_collision( eps );
        }
        double world_model_t::state_clearance( const sim::state_t* state )
        {
            selected_state_space->copy_from_point( state );
            return simulator->collision_distance();

        }

        collision_list_t* world_model_t::get_colliding_bodies()
        {
            return simulator->get_colliding_bodies();
        }

        collision_list_t* world_model_t::get_near_colliding_bodies( double eps )
        {
            return simulator->get_near_colliding_bodies( eps );
        }

        void world_model_t::steering_function(const state_t* start, const state_t* goal, plan_t& result)
        {
//            selected_state_space->copy_from_point(start);
            this->push_state(start);
            space_t* control_space = contexts[context_name].planning_control_space;
            state_t* full_start = simulator->get_state_space()->alloc_point();
//            selected_state_space->copy_from_point(goal);
            this->push_state(goal);
            state_t* full_goal = simulator->get_state_space()->alloc_point();
            plan_t full_result(simulator->get_control_space());
            control_t* control = control_space->alloc_point();
            //    simulator->steering_function(start,goal,full_result);
            simulator->steering_function(full_start, full_goal, full_result);
            foreach(plan_step_t step, full_result)
            {
                simulator->get_control_space()->copy_from_point(step.control);
                control_space->copy_to_point(control);
                result.copy_onto_back(control, step.duration);
            }
            simulator->get_state_space()->free_point(full_start);
            simulator->get_state_space()->free_point(full_goal);
            control_space->free_point(control);
        }

        control_t* world_model_t::compute_contingency_control(const state_t* curr_state)
        {
            PRX_ERROR_S("World model compute contingency controls isn't full featured. Using zero control.");
            control_t* control = selected_control_space->alloc_point();
            selected_control_space->zero(control);
            return control;
        }

        system_ptr_t world_model_t::get_system(const std::string& path) const
        {
            return simulator->get_system(path);
        }

        void world_model_t::update_sensing()
        {
            if (simulator->get_sensing_model() != NULL)
            {
                std::vector<sensing_info_t*> s_info = simulator->get_sensing_info();
                foreach(sensing_info_t* s, s_info)
                {
                    s->update_info();
                }
            }
        }

        void world_model_t::update_ground_truth(const double time, const std::vector<double>& elements)
        {

            space_t* full_state_space = (space_t*)simulator->get_state_space();

            for( unsigned i = 0; i < full_state_space->get_dimension(); i++ )
            {
                full_state->at(i) = elements[i];
            }

            simulator->push_state(full_state);
        }

        void world_model_t::init_collision_list(const parameter_reader_t * const reader)
        {
            std::string plant_name;

            vector_collision_list_t* black_list = new vector_collision_list_t();
            hash_t<std::string, system_ptr_t> obstacles = simulator->get_obstacles();

            foreach(std::string obst, obstacles | boost::adaptors::map_keys)
            PRX_DEBUG_S("Obstacle : " << obst);

            hash_t<std::string, plant_t*> plants;
            system_graph_t sys_graph;
            simulator->update_system_graph(sys_graph);
            sys_graph.get_path_plant_hash(plants);

            foreach(std::string name1, plants | boost::adaptors::map_keys)
            PRX_DEBUG_S("Plant: " << name1);

            int index = 0;

            if( reader->has_attribute("black_list") )
            {

                foreach(const parameter_reader_t* list_reader, reader->get_list("black_list"))
                {
                    index++;
                    std::vector<const parameter_reader_t*> r = list_reader->get_list("");

                    if( r.size() != 2 )
                        PRX_FATAL_S("The pair "<<index<<" in black list is wrong. Has to be a system with a list of systems.");

                    plant_name = r[0]->get_attribute("");

                    foreach(const parameter_reader_t* systems_reader, r[1]->get_list(""))
                    black_list->add_pair(plant_name, systems_reader->get_attribute(""));
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
                        PRX_FATAL_S("The pair "<<index<<" in white list is wrong. Has to be a system with a list of systems.");

                    plant_name = r[0]->get_attribute("");

                    foreach(const parameter_reader_t* systems_reader, r[1]->get_list(""))
                    {
                        std::string name2 = systems_reader->get_attribute("");
                        if( plants[name2] != NULL )
                            add_system_system_in_collision_list(plant_name, name2, black_list, plants);
                        else if( obstacles[name2] != NULL )
                            add_system_obstacle_in_collision_list(plant_name, name2, black_list, plants, obstacles);
                        else
                            PRX_WARN_S("The system/obstacle " << name2 << " does not exist in the simulation to pair with "<<plant_name<<".");
                    }
                }
            }
            else
            {
                PRX_DEBUG_S("Collision list initialized with all plants");
                collision_list_all_vs_all(plants, obstacles, black_list);
            }
            simulator->link_collision_list(collision_list);
            delete black_list;
        }

        void world_model_t::add_system_system_in_collision_list(const std::string& system1, const std::string& system2, const vector_collision_list_t* black_list, hash_t<std::string, plant_t*>& plants)
        {
            // SUPER DUPER HACK SUPER DUPER HACK SUPER DUPER HACK SUPER DUPER HACK 
            //                 IMPORTANT: APC CODE ONLY HACK!
            // SUPER DUPER HACK SUPER DUPER HACK SUPER DUPER HACK SUPER DUPER HACK 
            packages::baxter::manipulator_plant_t* firstman = dynamic_cast< packages::baxter::manipulator_plant_t* >( plants[system1] );
            packages::baxter::manipulator_plant_t* secondman = dynamic_cast< packages::baxter::manipulator_plant_t* >( plants[system2] );
            if( firstman != NULL && secondman != NULL )
            {
                return;
            }
            
            if( system1 != system2 && !black_list->pair_in_list(system1, system2) )
            {
                std::vector<std::string>* geom_names1 = plants[system1]->get_geometries_names();
                std::vector<std::string>* geom_names2 = plants[system2]->get_geometries_names();

                foreach(std::string n1, *geom_names1)
                {

                    foreach(std::string n2, *geom_names2)
                    {
                        collision_list->add_pair(n1, n2);
                    }
                }

                plants[system1]->update_collision_list(collision_list);
                plants[system2]->update_collision_list(collision_list);
            }
        }

        void world_model_t::add_system_obstacle_in_collision_list(const std::string& system, const std::string& obstacle, const vector_collision_list_t* black_list, hash_t<std::string, plant_t*>& plants, hash_t<std::string, system_ptr_t>& obstacles)
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

        void world_model_t::collision_list_all_vs_all(hash_t<std::string, plant_t*>& plants, hash_t<std::string, system_ptr_t>& obstacles, const vector_collision_list_t* black_list)
        {
            if(plants.size()==1)
            {
                plants[plants.begin()->first]->update_collision_list(collision_list);
            }
            for( hash_t<std::string, plant_t*>::const_iterator it = plants.begin(); it != plants.end(); it++ )
            {
                hash_t<std::string, plant_t*>::const_iterator temp_it = it;
                temp_it++;
                for( hash_t<std::string, plant_t*>::const_iterator it2 = temp_it; it2 != plants.end(); it2++ )
                {
                    add_system_system_in_collision_list(it->first, it2->first, black_list, plants);
                }

                foreach(std::string obst, obstacles | boost::adaptors::map_keys)
                {
                    add_system_obstacle_in_collision_list(it->first, obst, black_list, plants, obstacles);
                }
            }
        }

        void world_model_t::use_context(std::string name)
        {
            if( context_name != name )
            {
                foreach(std::string system_name, contexts[name].activation_list | boost::adaptors::map_keys)
                {
                    // PRX_DEBUG_S(system_name<<" "<<contexts[name].activation_list[system_name]);
                    simulator->set_active(contexts[name].activation_list[system_name], split_path(system_name).second);
                }
                context_name = name;
                selected_state_space = contexts[context_name].planning_state_space;
                selected_control_space = contexts[context_name].planning_control_space;
            }
        }
        std::string world_model_t::get_current_context()
        {
            return context_name;
        }

        bool world_model_t::check_context_name(std::string name)
        {
            return contexts.find(name) != contexts.end();
        }

        system_ptr_t world_model_t::get_split_system(const std::string& name) const
        {
            std::string split = split_path(name).second;
            return systems[split];
        }

        sim::system_graph_t& world_model_t::get_system_graph()
        {
            return sys_graph;
        }

        void world_model_t::set_propagate_response(bool resp)
        {
            respond = resp;
        }
        void world_model_t::create_new_planning_context(std::string new_context_name, util::hash_t<std::string, context_flags>& mappings, context_flags default_flags )
        {
            //This function will not be able to create actual embedded spaces, only the subset spaces.

            PRX_DEBUG_S("Creating a planning context: "<<new_context_name);

            //initialize the activation list for this space
            if(default_flags.plannable && !default_flags.active)
            {
                PRX_FATAL_S("Trying to create a planning context with unsatisfiable default flags. (Can't have plannable systems that are inactive)");
            }

            foreach(std::string name, systems | boost::adaptors::map_keys)
            {
                contexts[new_context_name].activation_list[name] = true;
            }


            std::vector<system_ptr_t> system_pointers;
            sys_graph.get_systems(system_pointers);
            //new_context_name is the name of the space to create
            std::vector<const space_t*> planning_spaces;
            std::vector<const space_t*> planning_controls;
            std::vector<const space_t*> active_spaces;
            std::vector<const space_t*> inactive_spaces;
            foreach(system_ptr_t system_pointer, system_pointers)
            {
                controller_t* system = dynamic_cast<controller_t*>(system_pointer.get());
                router_t* router_check = dynamic_cast<router_t*>(system_pointer.get());
                space_t* space = NULL;
                bool do_work = false;
                bool specialized_controls = false;
                std::pair<unsigned, unsigned> control_interval = get_full_control_space()->get_subspace(system_pointer.get()->get_control_space());

                if( router_check == NULL )
                {
                    //if a controller has its own state
                    if( system != NULL && (space = system->get_controller_state_space()) != NULL )
                    {
                        do_work = true;
                    }
                    //if the controller doesn't have its own state, but has controls that contribute
                    else if( system != NULL && control_interval.first != control_interval.second )
                    {
                        specialized_controls = true;
                    }
                    //if a plant, always consider it.
                    else if( system == NULL )
                    {
                        space = const_cast<space_t*>(system_pointer.get()->get_state_space());
                        do_work = true;
                    }
                }

                if( do_work )
                {
                    context_flags flags = default_flags;
                    //based on the type, we will add to the correct list
                    if(mappings.find(system_pointer.get()->get_pathname())!=mappings.end())
                        flags = mappings[system_pointer.get()->get_pathname()];

                    //based on the type, we need to do other behavior
                    if( !flags.plannable && !flags.active )
                    {
                        inactive_spaces.push_back(space);
                        if( system == NULL )
                        {
                            contexts[new_context_name].activation_list[system_pointer.get()->get_pathname()] = false;
                        }
                    }
                    else if( !flags.plannable && flags.active )
                    {
                        active_spaces.push_back(space);
                    }
                    else if( flags.plannable && flags.active )
                    {
                        planning_spaces.push_back(space);
                        if( control_interval.first != control_interval.second )
                        {
                            planning_controls.push_back(system_pointer.get()->get_control_space());
                        }
                    }
                    else
                    {
                        PRX_FATAL_S("Trying to create a planning context with unsatisfiable flags. (Can't have plannable systems that are inactive)");
                    }
                }
                else if( specialized_controls )
                {
                    // construct the control space embedding
                    planning_controls.push_back(system_pointer.get()->get_control_space());
                }
            }
            contexts[new_context_name].planning_state_space = new space_t(planning_spaces);
            contexts[new_context_name].planning_control_space = new space_t(planning_controls);
            contexts[new_context_name].active_space = new space_t(active_spaces);
            contexts[new_context_name].inactive_space = new space_t(inactive_spaces);
            contexts[new_context_name].temp_state = contexts[new_context_name].planning_state_space->alloc_point();
        }

        util::hash_t<std::string, system_ptr_t>& world_model_t::get_obstacles()
        {
            return simulator->get_obstacles();
        }

    }
}
