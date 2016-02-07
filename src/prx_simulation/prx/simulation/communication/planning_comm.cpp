/**
 * @file planning_comm.cpp 
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

#include "prx/simulation/communication/planning_comm.hpp"
#include "prx/simulation/applications/application.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/simulation/systems/controllers/consumer/consumer_controller.hpp"

#ifdef OCTAVE_FOUND
//#include "prx/simulation/systems/controllers/LQR_controller/LQR_tree.hpp"
#endif

#include "prx_simulation/lqr_msg.h"
#include "prx_simulation/plan_msg.h"
#include "prx_simulation/query_msg.h"

#include <boost/range/adaptor/map.hpp> //adaptors

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::comm::planning_comm_t, prx::sim::sim_base_communication_t)

namespace prx
{
    using namespace util;

    namespace sim
    {
        namespace comm
        {

            sim_base_communication_t* plan_comm;

            planning_comm_t::planning_comm_t()
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
                lqr_service = node.advertiseService(node_name + "/send_lqr", &planning_comm_t::lqr_callback, this);
                plant_point = NULL;
                plant_space = NULL;
            }

            planning_comm_t::~planning_comm_t()
            {
                if( plant_point )
                {
                    if( plant_space )
                    {
                        plant_space->free_point(plant_point);
                    }
                }
            }

            /**
             *
             */
            void planning_comm_t::set_plan(const std::string& system_name, const std::vector<prx_simulation::control_msg>& plan, const std::vector<double>& e_state)
            {
                simulator_t* sim = app->get_simulator();
                PRX_ASSERT(sim != NULL);
                //    PRX_ERROR_S ("Getting pointer to")
                system_ptr_t smart_sys = sim->get_system(system_name);

                system_t* sys = smart_sys.get();

                if( sys == NULL )
                    PRX_FATAL_S("Can't set plan for system " << system_name.c_str() << ". Not found!");

                consumer_controller_t* controller = checked_cast<consumer_controller_t*>(sys);
                if( controller == NULL )
                    PRX_FATAL_S("System " << system_name.c_str() << " does not have a consumer_controller_t!");

                plan_t new_plan(controller->get_output_control_space());
                const space_t* state_space = controller->get_subsystems()->begin()->second.get()->get_state_space();
                new_plan.link_state_space(state_space);
                control_t* control = controller->get_output_control_space()->alloc_point();

                foreach(const control_msg& control_msg, plan)
                {
                    for( unsigned int i = 0; i < control_msg.control.size(); ++i )
                        control->at(i) = control_msg.control[i];
                    new_plan.copy_onto_back(control, control_msg.duration);
                }
                controller->get_output_control_space()->free_point(control);

                if( e_state.size() > 0 )
                {
                    state_t* end_state = state_space->alloc_point();
                    for( unsigned i = 0; i < state_space->get_dimension(); i++ )
                    {
                        end_state->at(i) = e_state[i];
                    }
                    new_plan.copy_end_state(end_state);
                    state_space->free_point(end_state);
                }
                else
                {
                    PRX_WARN_S("End state is NULL (has not been set by planning");
                }
                //    controller->get_subsystems()->begin()->second.get()->get_state_space()->set_from_vector(e_state, end_state);

                PRX_DEBUG_COLOR("Received plan: \n" << new_plan.print(), PRX_TEXT_MAGENTA);
                //    PRX_INFO_S("Adding to plan for " << system_name
                //               << " - steps:" << new_plan.steps.size()
                //               << " - time:" << new_plan.length());
                controller->copy_plan(new_plan);
                //    PRX_WARN_S("This took: " <<timer.measure());
            }

            void planning_comm_t::set_lqr(const std::string& controller_name, const std::vector<prx_simulation::lqr_msg>& all_lqrs)
            {
#ifdef OCTAVE_FOUND
                //    std::vector<double> radii;
                //    std::vector<vector_t> centers;
                //    std::vector<vector_t> costs;
                //    std::vector<vector_t> gains;
                //    std::vector<vector_t> controls;
                //
                //    simulator_t* sim = app->get_simulator();
                //    PRX_ASSERT(sim != NULL);
                //
                //    system_ptr_t smart_sys = sim->get_system(controller_name);
                //
                //    system_t* sys = smart_sys.get();
                //
                //    if( sys == NULL )
                //        PRX_FATAL_S("Can't set lqr for system " << controller_name.c_str() << ". Not found!");
                //
                //    LQR_tree_t* controller = checked_cast<LQR_tree_t*>(sys);
                //    if( controller == NULL )
                //        PRX_FATAL_S("System " << controller_name.c_str() << " does not have a LQR_tree_t!");
                //
                //    foreach(const lqr_msg& lqr_msg, all_lqrs)
                //    {
                //        radii.push_back(lqr_msg.radius);
                //        vector_t new_center(lqr_msg.center.size());
                //        vector_t new_gain(lqr_msg.gain_matrix.size());
                //        vector_t new_cost(lqr_msg.cost.size());
                //        vector_t new_control(lqr_msg.control.size());
                //        
                //        for(unsigned i=0;i<new_center.get_dim();i++)
                //        {
                //            new_center[i] = lqr_msg.center[i];
                //        }
                //        for(unsigned i=0;i<new_gain.get_dim();i++)
                //        {
                //            new_gain[i] = lqr_msg.gain_matrix[i];
                //        }
                //        for(unsigned i=0;i<new_cost.get_dim();i++)
                //        {
                //            new_cost[i] = lqr_msg.cost[i];
                //        }
                //        for(unsigned i=0;i<new_control.get_dim();i++)
                //        {
                //            new_control[i] = lqr_msg.control[i];
                //        }
                //        centers.push_back(new_center);
                //        gains.push_back(new_gain);
                //        costs.push_back(new_cost);
                //        controls.push_back(new_control);
                //    }
                //    
                //    controller->set_internal_gains(radii,centers,controls,gains,costs);
#endif
#ifndef OCTAVE_FOUND
                PRX_INFO_S("This function does nothing without Octave");
#endif
            }

            /**
             *
             */
            void planning_comm_t::plans_callback(const prx_simulation::plan_msg& msg)
            {
                //    PRX_LOG_DEBUG ("Simulation plans callback");
                //    for (unsigned i = 0; i < msg.end_state.size(); i++)
                //    {
                //        PRX_ERROR_S ("End state " << i << ": " << msg.end_state[i]);
                //    }
                set_plan(msg.system_name, msg.plan, msg.end_state);
            }

            /**
             *
             */
            bool planning_comm_t::lqr_callback(send_lqr_srv::Request& request, send_lqr_srv::Response& response)
            {
                set_lqr(request.system_name, request.all_lqrs);
                return true;
            }

            void planning_comm_t::publish_planning_query(const std::vector<double>& start_state, const std::vector<double>& goal_state,
                                                         double goal_radius, const std::string& consumer_path, const std::string& planning_node, bool send_ground_truth, bool smooth,
                                                         bool set_goal_criterion, bool homogeneous_setup, double duration)
            {
                //    PRX_INFO_S ("Publish planning query! with radius: " << goal_radius);
                query_msg msg;
                PRX_DEBUG_COLOR("Putting start state into message", PRX_TEXT_GREEN);
                msg.goal_region_radius = goal_radius;
                msg.set_goal_criterion = set_goal_criterion;
                for( unsigned int i = 0; i < start_state.size(); i++ )
                {
                    msg.start.push_back(start_state[i]);
                    PRX_DEBUG_COLOR("Start state " << i << " : " << start_state[i], PRX_TEXT_GREEN);
                }

                //    PRX_DEBUG_S ("Putting goal state into message");

                for( unsigned int i = 0; i < goal_state.size(); i++ )
                {
                    msg.goal.push_back(goal_state[i]);
                    PRX_DEBUG_COLOR("Goal state " << i << " : " << goal_state[i], PRX_TEXT_GREEN);
                }

                msg.consumer = consumer_path;
                msg.smooth_plan = smooth;
                simulator_t* sim = app->get_simulator();
                if( send_ground_truth )
                {
                    system_graph_t subgraph;
                    sim->update_system_graph(subgraph);
                    hash_t<std::string, plant_t*> plants;
                    subgraph.get_path_plant_hash(plants);


                    //get the state space intervals
                    //const space_t* simulator_state_space = sim->get_state_space();

                    /** Only allocates one plant point. Assumes homogeneous setups*/
                    if( homogeneous_setup )
                    {
                        PRX_DEBUG_COLOR("Homogeneous setup", PRX_TEXT_GREEN);
                        if( plant_point == NULL )
                        {
                            plant_space = (*plants.begin()).second->get_state_space();
                            plant_point = plant_space->alloc_point();
                        }

                        foreach(plant_t* some_plant, plants | boost::adaptors::map_values)
                        {
                            PRX_DEBUG_COLOR("Some plant: " << some_plant->get_pathname(), PRX_TEXT_GREEN);
                            const space_t* subspace = some_plant->get_state_space();
                            subspace->copy_to_point(plant_point);
                            prx_simulation::state_msg some_plants_msg;

                            for( unsigned i = 0; i < subspace->get_dimension(); i++ )
                            {
                                some_plants_msg.elements.push_back(plant_point->at(i));
                                //                PRX_WARN_S ("Value: " << plant_point->at(i));
                            }
                            msg.plant_locations.plant_states.push_back(some_plants_msg);
                            msg.plant_locations.plant_paths.push_back(some_plant->get_pathname());

                        }
                    }
                    else
                    {
                        /** Allocates a plant point for each plant. */
                        PRX_DEBUG_COLOR("Non-homogeneous setup", PRX_TEXT_BLUE);

                        foreach(plant_t* some_plant, plants | boost::adaptors::map_values)
                        {
                            plant_space = some_plant->get_state_space();
                            plant_point = plant_space->alloc_point();
                            PRX_DEBUG_COLOR("Some plant: " << some_plant->get_pathname(), PRX_TEXT_GREEN);
                            const space_t* subspace = some_plant->get_state_space();
                            subspace->copy_to_point(plant_point);
                            prx_simulation::state_msg some_plants_msg;

                            for( unsigned i = 0; i < subspace->get_dimension(); i++ )
                            {
                                some_plants_msg.elements.push_back(plant_point->at(i));
                                PRX_DEBUG_COLOR("Value: " << plant_point->at(i), PRX_TEXT_GREEN);
                            }
                            msg.plant_locations.plant_states.push_back(some_plants_msg);
                            msg.plant_locations.plant_paths.push_back(some_plant->get_pathname());
                            plant_space->free_point(plant_point);
                            plant_space = NULL;

                        }
                    }
                    msg.plant_locations.node_name = planning_node;
                }

                planning_node_to_queries_topic[planning_node].publish(msg);
                PRX_DEBUG_COLOR("Planning query published!", PRX_TEXT_GREEN);
            }

            void planning_comm_t::create_planning_subscription(const std::string& planning_node)
            {

                if( plans_subscription.find(planning_node) == plans_subscription.end() )
                {
                    plans_subscription[planning_node] = node.subscribe(planning_node + "/plans", 60, &planning_comm_t::plans_callback, this);
                }
            }

            void planning_comm_t::create_new_query_topic(const std::string& planning_node)
            {
                if( planning_node_to_queries_topic.find(planning_node) == planning_node_to_queries_topic.end() )
                {
                    std::string node_name = ros::this_node::getName();
                    planning_node_to_queries_topic[planning_node] = node.advertise<prx_simulation::query_msg > (node_name + planning_node + "/planning_queries", 1, true);
                    PRX_DEBUG_COLOR("Created query publisher: " << node_name + planning_node + "/planning_queries", PRX_TEXT_CYAN);
                }
            }

        }
    }
}
