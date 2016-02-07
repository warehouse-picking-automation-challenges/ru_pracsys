/**
 * @file simulation_comm.cpp 
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

#include "prx/simulation/communication/simulation_comm.hpp"
#include "prx/simulation/applications/application.hpp"
#include "prx/simulation/systems/controllers/controller.hpp"
#include "prx/simulation/systems/controllers/consumer/consumer_controller.hpp"
#include "prx/simulation/systems/controllers/router.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/utilities/math/configurations/vector.hpp"
#include "prx_simulation/state_msg.h"
#include "prx_simulation/control_msg.h"
#include "prx_simulation/plant_locations_msg.h"


#include <boost/range/adaptor/map.hpp> //adaptors

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::comm::simulation_comm_t, prx::sim::sim_base_communication_t)

namespace prx
{
    using namespace util;

    namespace sim
    {
        namespace comm
        {
            sim_base_communication_t* sim_comm;

            simulation_comm_t::simulation_comm_t()
            {
                app = NULL;
                XmlRpc::XmlRpcValue sim_auto_send;
                node.getParam("sim_auto_send", sim_auto_send);
                if( sim_auto_send.getType() == XmlRpc::XmlRpcValue::TypeArray )
                {
                    for( int32_t i = 0; i < sim_auto_send.size(); ++i )
                    {
                        PRX_ASSERT(sim_auto_send[i].getType() == XmlRpc::XmlRpcValue::TypeString);
                        const std::string node_name = static_cast<std::string>(sim_auto_send[i]);
                    }
                }
                else
                {
                    PRX_WARN_S("No sim_auto_send parameter found, using default.");
                }

                std::string node_name = ros::this_node::getName();
                ground_truth_topic = node.advertise<prx_simulation::state_msg > (node_name + "/ground_truth", 10);
                request_ground_truth_service = node.advertiseService(node_name + "/request_ground_truth", &simulation_comm_t::request_ground_truth_callback, this);
                request_space_information_service = node.advertiseService(node_name + "/request_space_information", &simulation_comm_t::request_space_information_callback, this);
                set_sim_state_service = node.advertiseService(node_name + "/set_sim_state", &simulation_comm_t::set_sim_state_callback, this);

                //    plant_point = NULL;
                //    plant_space = NULL;
            }

            simulation_comm_t::~simulation_comm_t() {
                //    if (plant_point != NULL)
                //    {
                //        plant_space->free_point(plant_point);
                //    }
            }

            bool simulation_comm_t::request_ground_truth_callback(request_ground_truth_srv::Request& request, request_ground_truth_srv::Response& response)
            {
                PRX_DEBUG_COLOR("Request ground truth callback", PRX_TEXT_GREEN);
                //    app->add_ground_truth_query(request.timestamp_request, request.node_name);
                return true;
            }

            bool simulation_comm_t::set_sim_state_callback(set_sim_state_srv::Request& request, set_sim_state_srv::Response& response)
            {
                PRX_DEBUG_COLOR("Me: " << ros::this_node::getName() << " vs: " << request.node_name, PRX_TEXT_CYAN);
                if( ros::this_node::getName() == request.node_name )
                {
                    PRX_DEBUG_COLOR("Received state to set", PRX_TEXT_GREEN);
                    app->set_sim_state(request.sim_state);
                    return true;
                }
                else
                {
                    PRX_WARN_S("Attempted to set another simulation's state: " << request.node_name);
                    return false;
                }
            }

            //void simulation_comm_t::publish_state_spaces()
            //{
            //    PRX_ERROR_S("Publishing state_space: num systems: " << systems.size());
            //    prx_utilities::space_msg msg;
            //    controller_t* ctrl;
            //    for (unsigned int i = 0; i < systems.size(); i++)
            //    {
            //        PRX_ERROR_S ("Checking: " << systems[i]->get_pathname());
            //        ctrl = dynamic_cast<controller_t*> (systems[i].get());
            //        if (!ctrl)
            //        {
            //            PRX_INFO_S(" NOT a controller!");
            //            msg.system_path.push_back(systems[i]->get_pathname());
            //            msg.dimension.push_back(systems[i]->pull_state_space()->get_dimension());
            //        }
            //        
            //        //TODO: Temporary for now, until we have a merged controller interface
            //
            //    }
            //    
            //    state_space_topic.publish(msg);
            //}

            bool simulation_comm_t::request_space_information_callback(request_space_information_srv::Request& request, request_space_information_srv::Response& response)
            {
                PRX_DEBUG_COLOR("Request space information callback", PRX_TEXT_GREEN);
                simulator_t* sim = app->get_simulator();
                PRX_ASSERT(sim != NULL);

                //    PRX_ERROR_S ("Getting pointer to")

                //    if (std::strcmp("ground_truth", request.consumer_pathname.c_str()) == 0)
                if( request.consumer_pathname == "ground_truth" )
                {
                    //        system_ptr_t controller = sim;//->get_system(request.consumer_pathname);

                    //        controller_t* controller = dynamic_cast<consumer_controller_t*>(smart_sys.get());

                    //        if( controller == NULL )
                    //            PRX_LOG_ERROR("System %s is not a consumer_controller_t!", request.consumer_pathname.c_str());

                    system_graph_t subgraph;
                    sim->update_system_graph(subgraph);
                    std::vector<plant_t*> plants;
                    //SGC: only need plant pointers (in conjunction with the later todo, should be all systems)
                    subgraph.get_plants(plants);

                    //TODO: Requesting ground truth should not only consider plants (controllers with state).
                    //get the state space intervals
                    const space_t* simulator_state_space = sim->get_state_space();

                    foreach(plant_t* some_plant, plants)
                    {
                        PRX_DEBUG_COLOR("Some plant: " << some_plant->get_pathname(), PRX_TEXT_GREEN);
                        const space_t* subspace = some_plant->get_state_space();
                        std::pair<unsigned, unsigned> sys_interval = simulator_state_space->get_subspace(subspace);
                        interval_msg state_interval;
                        state_interval.lower = sys_interval.first;
                        state_interval.upper = sys_interval.second;
                        state_interval.pathname = some_plant->get_pathname();
                        response.state_space_intervals.push_back(state_interval);

                        interval_msg control_interval;
                        control_interval.lower = 0;
                        control_interval.upper = 0;
                        control_interval.pathname = some_plant->get_pathname();
                        response.control_space_intervals.push_back(control_interval);
                    }

                }
                else
                {

                    //TODO: make routers work under consumers
                    system_ptr_t smart_sys = sim->get_system(request.consumer_pathname);
                    PRX_DEBUG_S(request.consumer_pathname);

                    consumer_controller_t* controller = dynamic_cast<consumer_controller_t*>(smart_sys.get());

                    if( controller == NULL )
                        PRX_FATAL_S("System " << request.consumer_pathname.c_str() << " is not a consumer_controller_t!");

                    system_graph_t subgraph;
                    controller->update_system_graph(subgraph);
                    std::vector<system_ptr_t> subsystems;
                    subgraph.get_systems(subsystems);
                    PRX_DEBUG_COLOR("NUMBER OF SYSTEMS: " << subsystems.size(), PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("NUMBER OF SYSTEMS: " << subsystems[0].get(), PRX_TEXT_GREEN);
                    PRX_DEBUG_COLOR("NUMBER OF SYSTEMS: " << subsystems[1].get(), PRX_TEXT_GREEN);


                    //get the state space intervals
                    const space_t* consumer_state_space = controller->get_state_space();

                    foreach(system_ptr_t subsystem, subsystems)
                    {
                        //If something is broken with the space information then bring this if back.
                        //            if(subsystem.get()->get_pathname() != controller->get_pathname())
                        controller_t* casted_controller = dynamic_cast<controller_t*>(subsystem.get());
                        const space_t* subspace;
                        if( casted_controller == NULL )
                        {
                            //this needs to get the space of the subsystem, compute the interval, and create a message to put into response
                            subspace = subsystem.get()->get_state_space();
                        }
                        else
                        {
                            subspace = casted_controller->get_controller_state_space();
                        }

                        std::pair<unsigned, unsigned> sys_interval = consumer_state_space->get_subspace(subspace);
                        interval_msg state_interval;
                        if( sys_interval.first != sys_interval.second )
                        {
                            state_interval.lower = sys_interval.first;
                            state_interval.upper = sys_interval.second;
                            state_interval.pathname = subsystem.get()->get_pathname();
                            response.state_space_intervals.push_back(state_interval);
                            PRX_DEBUG_COLOR("Pathname: " << subsystem->get_pathname() << " Lower: " << sys_interval.first << " , Upper: " << sys_interval.second, PRX_TEXT_GREEN);
                        }

                    }

                    const space_t* consumer_control_space = controller->get_output_control_space();

                    foreach(system_ptr_t subsystem, subsystems)
                    {
                        //If something is broken with the space information then bring this if back.
                        //            if(subsystem.get()->get_pathname() != controller->get_pathname())
                        router_t* casted_controller = dynamic_cast<router_t*>(subsystem.get());
                        if( casted_controller == NULL )
                        {
                            const space_t* subspace = subsystem.get()->get_control_space();
                            std::pair<unsigned, unsigned> sys_interval = consumer_control_space->get_subspace(subspace);
                            interval_msg control_interval;
                            if( sys_interval.first != sys_interval.second )
                            {
                                control_interval.lower = sys_interval.first;
                                control_interval.upper = sys_interval.second;
                                control_interval.pathname = subsystem.get()->get_pathname();
                                response.control_space_intervals.push_back(control_interval);
                                PRX_DEBUG_COLOR("Pathname: " << subsystem->get_pathname() << " Lower: " << sys_interval.first << " , Upper: " << sys_interval.second, PRX_TEXT_GREEN);
                            }

                        }


                    }


                    // const hash_t<std::string, system_ptr_t >* direct_subsystems = controller->get_subsystems();

                    // foreach(system_ptr_t subsystem, (*direct_subsystems) | boost::adaptors::map_values)
                    // {
                    //     PRX_PRINT("Printin mah subsystem!" << subsystem.get()->get_pathname(), PRX_TEXT_CYAN);
                    //     //this needs to get the space of the subsystem, compute the interval, and create a message to put into response
                    //     const space_t* subspace = subsystem.get()->get_control_space();
                    //     std::pair<unsigned, unsigned> sys_interval = consumer_control_space->get_subspace(subspace);
                    //     interval_msg control_interval;
                    //     control_interval.lower = sys_interval.first;
                    //     control_interval.upper = sys_interval.second;
                    //     control_interval.pathname = subsystem.get()->get_pathname();
                    //     response.control_space_intervals.push_back(control_interval);
                    // }
                }

                return true;

            }

            void simulation_comm_t::publish_ground_truth(double time, const std::string& node_name, const std::string& consumer_name)
            {
                PRX_DEBUG_COLOR("Publishing ground truth-> Time: " << time << " , node_name: " << node_name << ", consumer name: " << consumer_name, PRX_TEXT_CYAN);
                prx_simulation::state_msg msg;
                msg.timestamp = time;
                msg.node_name = node_name;
                msg.consumer_name = consumer_name;

                simulator_t* simulator = app->get_simulator();
                const space_t* s_space = simulator->get_state_space();
                const state_t* sim_state = simulator->pull_state();

                if( consumer_name == "ground_truth" )
                {
                    for( unsigned int i = 0; i < s_space->get_dimension(); ++i )
                    {
                        msg.elements.push_back(sim_state->at(i));
                    }
                }
                else
                {
                    PRX_DEBUG_COLOR("Publishing ground truth under consumer: " << consumer_name, PRX_TEXT_BLUE);
                    const space_t* c_space = simulator->get_system(consumer_name)->get_state_space();
                    space_point_t* c_state = c_space->alloc_point();

                    //TODO: This is a hacky way to get rid of the consumer's time state
                    for( unsigned i = 0; i < c_space->get_dimension() - 1; ++i )
                    {
                        msg.elements.push_back(c_state->at(i));
                        PRX_WARN_S("Val: " << msg.elements.back());
                    }
                    c_space->free_point(c_state);

                }

                //    PRX_INFO_S ("Pushed back: " << s_space->print_point(sim_state));


                ground_truth_topic.publish(msg);
            }

            //void simulation_comm_t::create_new_plant_state_publisher(const std::string& planning_node)
            //{
            //        // Creates publisher to planning node if it does not exist
            //    if (planning_node_to_plant_locations_topic.find(planning_node) == planning_node_to_plant_locations_topic.end())
            //    {
            //        // TODO: This works only if a single simulation node is running, make it work for multiple simulation nodes
            //        planning_node_to_plant_locations_topic[planning_node] = node.advertise<prx_simulation::plant_locations_msg > ("simulation" + planning_node + "/plant_locations", 1);
            //        
            //        PRX_WARN_S ("Created plant location publisher: " << "simulation" + planning_node + "/plant_locations");
            //    }
            //}
            //
            ///**
            // *
            // */
            //void simulation_comm_t::publish_plant_states(const std::string& planning_node)
            //{
            //    PRX_ERROR_S ("Publish plant states!");
            //    simulator_t* sim = app->get_simulator();
            //    
            //    PRX_ASSERT(sim != NULL);
            //    
            //    prx_simulation::plant_locations_msg msg;
            //    system_graph_t subgraph;
            //    sim->update_system_graph(subgraph);
            //    hash_t<std::string, plant_t*> plants;
            //    subgraph.get_plants(plants);
            //
            //
            //    //get the state space intervals
            //    const space_t* simulator_state_space = sim->get_state_space();
            //    // OPTIMIZED for homogenous setups. 
            //    // TODO: Heteregenous setups
            //    if (plant_point == NULL)
            //    {
            //        plant_space = (*plants.begin()).second->get_state_space();
            //        plant_point = plant_space->alloc_point();
            //    }
            //    foreach(plant_t* some_plant, plants | boost::adaptors::map_values)
            //    {
            //        PRX_DEBUG_S ("Some plant: " << some_plant->get_pathname());
            //        const space_t* subspace = some_plant->get_state_space();
            //        subspace->copy_to_point(plant_point);
            //        prx_simulation::state_msg some_plants_msg;
            //        
            //        for (unsigned i = 0 ; i < subspace->get_dimension(); i++)
            //        {
            //            some_plants_msg.elements.push_back(plant_point->at(i));
            //            PRX_WARN_S ("Value: " << plant_point->at(i));
            //        }
            //        msg.plant_states.push_back(some_plants_msg);
            //        msg.plant_paths.push_back(some_plant->get_pathname());
            //        
            //    }
            //
            //    msg.node_name = planning_node;
            //    PRX_WARN_S ("Publishing!!!!!!!!!!");
            //    
            //    planning_node_to_plant_locations_topic[planning_node].publish(msg);
            //}

        }
    }
}
