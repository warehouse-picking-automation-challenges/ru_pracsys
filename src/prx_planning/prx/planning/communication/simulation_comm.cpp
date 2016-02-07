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

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/communication/simulation_comm.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/planning/applications/planning_application.hpp"
#include "prx_simulation/control_msg.h"
#include "prx_simulation/send_plans_srv.h"
#include "prx_simulation/request_ground_truth_srv.h"
#include "prx_simulation/set_sim_state_srv.h"


#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

PLUGINLIB_EXPORT_CLASS( prx::plan::comm::simulation_comm_t, prx::plan::plan_base_communication_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        namespace comm
        {
            plan_base_communication_t* sim_comm;


            using namespace comm;

            simulation_comm_t::simulation_comm_t()
            {
                planning_app = NULL;

            }
            
            void simulation_comm_t::link_application(planning_application_t* parent_app)
            {
                plan_base_communication_t::link_application(parent_app);
                
                std::string sim_node = planning_app->get_simulation_node_name();
                ground_truth_subscriber = node.subscribe(sim_node + "/ground_truth", 10,
                                                         &simulation_comm_t::update_ground_truth_callback, this);
                //    planning_query_subscriber = node.subscribe(sim_node + "/planning_queries", 10, 
                //                &simulation_comm_t::planning_query_callback,this);
                plant_locations_subscriber = node.subscribe(sim_node + ros::this_node::getName() + "/plant_locations", 1,
                                                            &simulation_comm_t::plant_locations_callback, this);
            }

            void simulation_comm_t::update_ground_truth_callback(const prx_simulation::state_msg& msg)
            {
                PRX_DEBUG_S("Update ground truth callback: msg name: " << msg.node_name << ", my name: " << ros::this_node::getName());
                if( msg.node_name == ros::this_node::getName() )
                {
                    PRX_ERROR_S("Updating ground truth is not implemented!!!!!");
                    //TODO : Shouldn't we actually be updating ground truth information?
                    //        planning_application->update_ground_truth(msg.timestamp, msg.elements, msg.consumer_path);
                }
            }

            // TODO: Deprecated
            //void simulation_comm_t::request_ground_truth(double time)
            //{
            //    ros::ServiceClient client = node.serviceClient<request_ground_truth_srv > ("simulation/request_ground_truth");
            //    request_ground_truth_srv srv;
            //    srv.request.timestamp_request = time;
            //    srv.request.node_name = ros::this_node::getName();
            //    client.waitForExistence(ros::Duration(5));
            //    // failed to  call service
            //    if( !client.call(srv) )
            //        PRX_LOG_ERROR("Failed to request ground truth update.");
            //}

            //void simulation_comm_t::planning_query_callback(const prx_simulation::query_msg& msg)
            //{
            //    //TODO : Shouldn't this function have implementation?
            //
            ////    std::string name = msg.planned_system;
            ////    PRX_DEBUG_S("Inside planning query  Callback: for system " << name);
            ////    
            ////    vector_t goal_state; goal_state.resize(msg.goal_state.size());
            ////    vector_t start_state; start_state.resize(msg.start_state.size());
            ////    for(int i = 0 ; i < msg.goal_state.size(); i++)
            ////    {
            ////        goal_state[i] = msg.goal_state[i];
            ////        start_state[i] = msg.start_state[i];
            ////    }
            ////    
            ////    sys_clock_t query_planner_timer; query_planner_timer.reset();
            //////    planning_application->query_planner(name, goal_state, start_state, msg.smooth, msg.time_limit);
            ////    PRX_ERROR_S ("Query time took: " << query_planner_timer.measure());
            //}

            void simulation_comm_t::plant_locations_callback(const prx_simulation::plant_locations_msg& msg)
            {
                // PRX_WARN_S("Sup plant locations callback?\n\n\n\n\n");

                //    if (msg.node_name == ros::this_node::getName() )
                planning_app->process_plant_locations_callback(msg);
            }
            
            void simulation_comm_t::set_simulator_state(const std::vector<double>& point)
            {
                set_sim_state_srv srv;
                
                foreach(double element, point)
                {
                    srv.request.sim_state.push_back(element);
                }
                std::string sim_node = planning_app->get_simulation_node_name();
                srv.request.node_name = sim_node;

                // Construct request
                PRX_DEBUG_S("Setting state for simulator: " << sim_node);
                ros::ServiceClient client = node.serviceClient<set_sim_state_srv > (sim_node + "/set_sim_state");
                client.waitForExistence(ros::Duration(-1));
                if( !client.call(srv) )
                    PRX_FATAL_S("Failed to set simulation state");

            }
        }
    }
}
