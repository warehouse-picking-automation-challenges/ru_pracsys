/**
 * @file planning_application.cpp 
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

#include "prx/planning/applications/planning_application.hpp"
#include "prx/planning/communication/plan_base_communication.hpp"
#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/communication/simulation_comm.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/problem_specifications/specification.hpp"
#include "prx/planning/queries/query.hpp"
#include "prx/planning/modules/stopping_criteria/element/criterion.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/utilities/communication/tf_broadcaster.hpp"

#include "prx_simulation/request_space_information_srv.h"


namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        using namespace comm;
        //using namespace prx_simulation;

        pluginlib::ClassLoader<planning_application_t> planning_application_t::loader("prx_planning", "prx::plan::planning_application_t");

        planning_application_t::planning_application_t()
        {
            model = NULL;
            tf_broadcaster = new tf_broadcaster_t();
            use_sys_mapping = false;
        }

        planning_application_t::~planning_application_t()
        {
            delete plan_comm;
            delete sim_comm;
            delete vis_comm;
            delete tf_broadcaster;
            if( model != NULL )
                delete model;
        }

        void planning_application_t::init(const parameter_reader_t* reader)
        {
            PRX_DEBUG_S("Initializing planning application...");
            simulation::update_all_configs = false;
            //debug and communication flags
            debug = reader->get_attribute_as<bool>("debug", false);
            visualize = reader->get_attribute_as<bool>("visualize", true);
            simulate = reader->get_attribute_as<bool>("simulate", true);
            if( simulate )
            {
                //TODO: Do we need to enforce a planning node's input to contain the simulation node's name it is communicating with?
                simulation_node_name = reader->get_attribute_as<std::string > ("simulation_node_name", "/simulation");
                PRX_DEBUG_S("This planning node will talk to simulation node " << simulation_node_name);
            }

            std::string plugin_type_name;
            plugin_type_name = reader->get_attribute("plan_comm", "planning_comm");
            plan_comm = plan_base_communication_t::get_loader().createUnmanagedInstance("prx_planning/" + plugin_type_name);
            plan_comm->link_application(this);

            plugin_type_name = reader->get_attribute("plan_to_sim_comm", "simulation_comm");
            sim_comm = plan_base_communication_t::get_loader().createUnmanagedInstance("prx_planning/" + plugin_type_name);
            sim_comm->link_application(this);

            plugin_type_name = reader->get_attribute("plan_to_vis_comm", "visualization_comm");
            vis_comm = plan_base_communication_t::get_loader().createUnmanagedInstance("prx_planning/" + plugin_type_name);
            vis_comm->link_application(this);

            model = new world_model_t();
            reader->initialize(model, "world_model");

            ((comm::visualization_comm_t*)comm::vis_comm)->to_config = boost::bind(&world_model_t::get_configs, model, _1, _2);

            root_task = (task_planner_t*)reader->create_from_loader<planner_t > ("task_planner", "prx_planning");
            root_task->set_pathname("task_planner");
            parameters::initialize(root_task, reader, "task_planner", NULL, "");
            // reader->initialize(*root_task,"task_planner");
            root_task->link_world_model(model);

            // If there is a 1-to-1 correspondance of state and control between simulation
            // and planning, then use the system mapping
            if( simulate && reader->has_attribute("system_mapping") )
            {
                std::vector<const parameter_reader_t*> sys_map_reader = reader->get_list("system_mapping");

                foreach(const parameter_reader_t* item_reader, sys_map_reader)
                {
                    std::vector<std::string> pair = item_reader->get_attribute_as<std::vector<std::string> >("pair");
                    this->simulator_to_planning_sys_mapping[pair[0]] = pair[1];
                    this->planning_to_simulator_sys_mapping[pair[1]] = pair[0];

                }
                use_sys_mapping = true;
            }
                /* Otherwise, use separate state and control mappings
                 */
            else if( simulate )
            {
                std::vector<const parameter_reader_t*> state_map_reader = reader->get_list("state_mapping");

                foreach(const parameter_reader_t* item_reader, state_map_reader)
                {
                    std::vector<std::string> pair = item_reader->get_attribute_as<std::vector<std::string> >("pair");
                    this->simulator_to_planning_sys_state_mapping[pair[0]] = pair[1];
                    this->planning_to_simulator_sys_mapping[pair[1]] = pair[0];
                    PRX_DEBUG_COLOR("Pair: " << pair[0] << " " << pair[1], PRX_TEXT_GREEN);

                }

                std::vector<const parameter_reader_t*> control_map_reader = reader->get_list("control_mapping");

                foreach(const parameter_reader_t* item_reader, control_map_reader)
                {
                    std::vector<std::string> pair = item_reader->get_attribute_as<std::vector<std::string> >("pair");
                    this->simulator_to_planning_sys_control_mapping[pair[0]] = pair[1];
                    PRX_DEBUG_S("Pair: " << pair[0] << " " << pair[1]);
                }

                use_sys_mapping = false;


            }

            // This mapping allows us to set the world model appropriately depending on the consumer
            if( simulate && reader->has_attribute("consumer_mapping") )
            {
                std::vector<const parameter_reader_t*> consumer_mapping = reader->get_list("consumer_mapping");

                foreach(const parameter_reader_t* item_reader, consumer_mapping)
                {
                    std::vector<std::string> pair = item_reader->get_attribute_as<std::vector<std::string> >("pair");
                    PRX_ASSERT(model->check_context_name(pair[1]));
                    this->consumer_to_space_name_mapping[pair[0]] = pair[1];
                    PRX_INFO_S("Paired consumer: " << pair[0] << " with space name: " << pair[1]);
                }
            }

            if( reader->has_attribute("problems") )
            {
                parameter_reader_t::reader_map_t problem_reader = reader->get_map("problems");

                const parameter_reader_t* query_template_reader = NULL;
                std::string query_template_name;
                const parameter_reader_t* specification_template_reader = NULL;
                std::string specification_template_name;

                foreach(const parameter_reader_t::reader_map_t::value_type problem_item, problem_reader)
                {
                    if( problem_item.second->has_attribute("specification/template") )
                    {
                        specification_template_name = problem_item.second->get_attribute("specification/template");
                        specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + specification_template_name);
                    }

                    specification_t* specification = parameters::initialize_from_loader<specification_t>("prx_planning", problem_item.second, "specification", specification_template_reader, "");

                    if( specification_template_reader != NULL )
                    {
                        delete specification_template_reader;
                        specification_template_reader = NULL;
                    }

                    root_specifications.push_back(specification);

                    if(problem_item.second->has_attribute("query"))
                    {
                        PRX_DEBUG_COLOR("Found query with specification", PRX_TEXT_BLUE);
                        if( problem_item.second->has_attribute("query/template") )
                        {
                            query_template_name = problem_item.second->get_attribute("query/template");
                            query_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + query_template_name);
                        }

                        query_t* query = parameters::initialize_from_loader<query_t>("prx_planning", problem_item.second, "query", query_template_reader, "");

                        if( query_template_reader != NULL )
                        {
                            delete query_template_reader;
                            query_template_reader = NULL;
                        }

                        root_queries.push_back(query);
                    }
                    else
                    {
                        PRX_WARN_S ("No query found in input. Pushing a NULL onto root_queries");
                        root_queries.push_back(NULL);
                    }
                }
            }
            else
            {
                PRX_WARN_S("No problem definitions read in from input!");
            }

            // Initialize spaces here if we have read in a consumer
            //    if (reader->has_attribute("consumer"))
            if( simulate )
            {
                this->consumer_path = reader->get_attribute("consumer");
                initialize_spaces(consumer_path);
            }

        }

        //void planning_application_t::update_ground_truth()
        //{
        ////    PRX_ERROR_S ("Update ground truth inside task planner: " << simulator_paths.size());
        //////    model->update_ground_truth(time,elements);
        ////    
        ////    if (state_space_set)
        ////    {
        ////        int counter = 0;
        ////        for (int i = 0; i < simulator_paths.size(); i++)
        ////        {
        ////            for (int j = 0; counter < simulator_path_to_offset[simulator_paths[i]]; j++)
        ////            {
        ////                system_state_space->set_element(simulator_path_to_state[simulator_paths[i]], j, elements[counter]);
        ////                counter++;
        ////            }
        ////            PRX_INFO_S ("Update " << simulator_paths[i] << " to : " << system_state_space->print_point(simulator_path_to_state[simulator_paths[i]]));
        ////        }
        ////    }
        ////    else
        ////    {
        ////        PRX_ERROR_S ("Update ground truth called without state spaces being initialized!");
        ////    }
        //}

        std::vector<double> planning_application_t::sim_to_plan_state(const std::vector<double>& state, std::string c_path)
        {
            if( c_path.empty() )
                c_path = consumer_path;
            PRX_ASSERT(consumer_path_to_state_intervals.find(consumer_path) != consumer_path_to_state_intervals.end());

            std::vector<double> new_state(state.size());

            foreach(interval_pair_t interval_pair, consumer_path_to_state_intervals[consumer_path])
            {
                for( unsigned s = interval_pair.first.first, p = interval_pair.second.first;
                     s < interval_pair.first.second && p < interval_pair.second.second;
                     s++, p++ )
                {
                    new_state[p] = state[s];
                }
            }
            return new_state;
        }

        std::vector<double> planning_application_t::plan_to_sim_state(state_t* state, std::string c_path)
        {
            if( c_path.empty() )
                c_path = consumer_path;
            PRX_ASSERT(consumer_path_to_state_intervals.find(consumer_path) != consumer_path_to_state_intervals.end());
            std::vector<double> new_state(model->get_state_space()->get_dimension());

            foreach(interval_pair_t interval_pair, consumer_path_to_state_intervals[consumer_path])
            {
                for( unsigned s = interval_pair.first.first, p = interval_pair.second.first;
                     s < interval_pair.first.second && p < interval_pair.second.second;
                     s++, p++ )
                {
                    new_state[s] = state->at(p);
                }
            }
            return new_state;
        }

        std::vector<double> planning_application_t::sim_to_plan_control(const std::vector<double>& control, std::string c_path)
        {
            if( c_path.empty() )
                c_path = consumer_path;
            PRX_ASSERT(consumer_path_to_control_intervals.find(consumer_path) != consumer_path_to_control_intervals.end());
            std::vector<double> new_control(control.size());

            foreach(interval_pair_t interval_pair, consumer_path_to_control_intervals[consumer_path])
            {
                for( unsigned s = interval_pair.first.first, p = interval_pair.second.first;
                     s < interval_pair.first.second && p < interval_pair.second.second;
                     s++, p++ )
                {
                    new_control[p] = control[s];
                }
            }
            return new_control;
        }

        std::vector<double> planning_application_t::plan_to_sim_control(control_t* control, std::string c_path)
        {
            if( c_path.empty() )
                c_path = consumer_path;
            PRX_ASSERT(consumer_path_to_control_intervals.find(consumer_path) != consumer_path_to_control_intervals.end());
            std::vector<double> new_control(model->get_control_space()->get_dimension());

            foreach(interval_pair_t interval_pair, consumer_path_to_control_intervals[consumer_path])
            {
                for( unsigned s = interval_pair.first.first, p = interval_pair.second.first;
                     s < interval_pair.first.second && p < interval_pair.second.second;
                     s++, p++ )
                {
                    new_control[s] = control->at(p);
                }
            }

            return new_control;
        }

        void planning_application_t::initialize_spaces(std::string consumer_path)
        {
            std::vector<prx_simulation::interval_msg> state_interval_msg, control_interval_msg;

            std::string old_context = model->get_current_context();
            // Request intervals from simulation
            if( simulate )
            {
                if( consumer_to_space_name_mapping.find(consumer_path) == this->consumer_to_space_name_mapping.end() )
                {
                    PRX_WARN_S("Consumer to space name mapping has not been set for " << consumer_path << " in planner structure input file!");
                }
                ((comm::planning_comm_t*)comm::plan_comm)->request_space_information(consumer_path, state_interval_msg, control_interval_msg);

                model->use_context(consumer_to_space_name_mapping[consumer_path]);
            }

            std::vector<interval_pair_t> new_state_interval, new_control_interval;

            // State intervals

            foreach(interval_msg msg, state_interval_msg)
            {
                if( msg.lower != msg.upper )
                {
                    // Create simulator's interval
                    interval_t sim_interval = std::make_pair<unsigned, unsigned>(msg.lower, msg.upper);

                    // Find the wm system and its space
                    std::string wm_sys_path;
                    if( use_sys_mapping )
                        wm_sys_path = this->simulator_to_planning_sys_mapping[msg.pathname];
                    else
                        wm_sys_path = this->simulator_to_planning_sys_state_mapping[msg.pathname];
                    PRX_PRINT(wm_sys_path,PRX_TEXT_RED);
                    if( wm_sys_path.empty() )
                    {
                        PRX_WARN_S("Tried to access the system mapping with " << msg.pathname << " which does not exist!");
                    }
                    else
                    {
                        system_ptr_t wm_sys = model->get_split_system(wm_sys_path);
                        const space_t* full_state_space = model->get_state_space();
                        const space_t* wm_sys_space = wm_sys.get()->get_state_space();

                        // Create wm's interval
                        interval_t wm_interval;
                        wm_interval = full_state_space->get_subspace(wm_sys_space);
                        interval_pair_t new_pair = std::make_pair<interval_t, interval_t > (sim_interval, wm_interval);
                        new_state_interval.push_back(new_pair);
                    }
                }
            }

            this->consumer_path_to_state_intervals[consumer_path] = new_state_interval;

            // Control intervals

            foreach(interval_msg msg, control_interval_msg)
            {
                // Create simulator's interval
                interval_t sim_interval = std::make_pair<unsigned, unsigned>(msg.lower, msg.upper);

                // Find the wm system and its space
                std::string wm_sys_path;
                if( use_sys_mapping )
                    wm_sys_path = this->simulator_to_planning_sys_mapping[msg.pathname];
                else
                    wm_sys_path = this->simulator_to_planning_sys_control_mapping[msg.pathname];
                PRX_PRINT(wm_sys_path,PRX_TEXT_BLUE);
                if( wm_sys_path.empty() )
                {
                    PRX_WARN_S("Tried to access the system mapping with " << msg.pathname << " which does not exist!");
                }
                else
                {
                    system_ptr_t wm_sys = model->get_split_system(wm_sys_path);
                    const space_t* full_control_space = model->get_control_space();
                    const space_t* wm_sys_space = wm_sys.get()->get_control_space();

                    // Create wm's interval
                    interval_t wm_interval;
                    wm_interval = full_control_space->get_subspace(wm_sys_space);
                    interval_pair_t new_pair = std::make_pair<interval_t, interval_t > (sim_interval, wm_interval);
                    new_control_interval.push_back(new_pair);
                }
            }

            this->consumer_path_to_control_intervals[consumer_path] = new_control_interval;

            //    this->consumer_path = consumer_path;

            model->use_context(old_context);

        }

        void planning_application_t::process_query(const prx_simulation::query_msg& msg)
        {
            this->process_query_callback(msg);
        }

        void planning_application_t::process_ground_truth(const prx_simulation::state_msg& msg)
        {
            this->process_ground_truth_callback(msg);
        }

        void planning_application_t::process_query_callback(const prx_simulation::query_msg& msg)
        {

            PRX_ERROR_S("Process query callback has no default implementation");
            //    // We've received a query!
            //    
            //    
            //    
            //    // Do root_tasks...
            //    if (this->consumer_path_to_state_intervals.find(consumer_path) != consumer_path_to_state_intervals.end())
            //    {
            //        initialize_spaces(consumer_path);
            //    }
            //    
            //    // Create a query ?
            //    
            //    // Send to task planner
            ////    root_task->link_query(root_tasky);
        }

        void planning_application_t::process_ground_truth_callback(const prx_simulation::state_msg& msg)
        {
            PRX_ERROR_S("Process ground truth callback has no default implementation");

            //    PRX_DEBUG_S("Inside planning query  Callback: for system " << request.planned_system);
            //    
            //    std::string name = request.planned_system;
            //    vector_t goal_state; goal_state.resize(request.goal_state.size());
            //    vector_t start_state; start_state.resize(request.start_state.size());
            //    for(int i = 0 ; i < request.goal_state.size(); i++)
            //    {
            //        goal_state[i] = request.goal_state[i];
            //        start_state[i] = request.start_state[i];
            //    }
            //    
            //    sys_clock_t query_planner_timer; query_planner_timer.reset();
            ////    app->query_planner(name, goal_state, start_state, request.smooth, request.time_limit);
            //    PRX_ERROR_S ("Query time took: " << query_planner_timer.measure());
            //    return true;
        }

        void planning_application_t::process_plant_locations_callback(const prx_simulation::plant_locations_msg& msg)
        {
            PRX_ERROR_S("Process plant locations callback not implemented by default!"); // TODO : Why then is there a default implementation here?

            // foreach(prx_simulation::state_msg root_task, msg.plant_states)
            // {
            //     PRX_DEBUG_S("New plant: ");

            //     foreach(double val, root_task.elements)
            //     {
            //         PRX_DEBUG_S("Val : " << val);
            //     }
            // }
        }

        void planning_application_t::process_graph_callback(const prx_simulation::graph_msg& msg)
        {
            PRX_ERROR_S("Process graph callback has no default implementation");
        }

        void planning_application_t::process_ack_callback(const prx_simulation::manipulation_acknowledgement& msg)
        {
            PRX_ERROR_S("Process acknowledgement callback has no default implementation");
        }

        pluginlib::ClassLoader<planning_application_t>& planning_application_t::get_loader()
        {
            return loader;
        }

        std::string planning_application_t::get_simulation_node_name() const
        {
            PRX_DEBUG_S("returning simulation node name: " << simulation_node_name);
            return simulation_node_name;
        }

        bool planning_application_t::is_visualizing() const
        {
            return visualize;
        }


    }
}
