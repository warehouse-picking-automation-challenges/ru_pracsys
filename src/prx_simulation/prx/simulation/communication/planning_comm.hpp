/**
 * @file planning_comm.hpp 
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
#pragma once

#ifndef PRX_SIMULATION_PLANNING_COMM_HPP
#define	PRX_SIMULATION_PLANNING_COMM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/communication/sim_base_communication.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/systems/system.hpp"

#include "prx_simulation/lqr_msg.h"
#include "prx_utilities/planning_query_msg.h"
#include "prx_simulation/send_plans_srv.h"
#include "prx_simulation/send_lqr_srv.h"
#include "prx_simulation/state_msg.h"
#include "prx_simulation/plan_msg.h"
#include "prx_simulation/plant_locations_msg.h"

#include <std_msgs/Int32.h>

#include <ros/ros.h>


namespace prx
{

    namespace util
    {
        class vector_t;
    }

    namespace sim
    {

        using namespace prx_utilities;
        using namespace prx_simulation;

        class application_t;

        namespace comm
        {

            /**
             * @anchor planning_comm_t
             *
             * This class is responsible for receiving messages and service calls
             * specifically from the planning node.
             *
             * @brief <b> Simulation node's communication class for talking to planning. </b>
             * 
             * @author Andrew Kimmel
             */
            class planning_comm_t : public sim_base_communication_t
            {

              protected:

                // Advertised services
                // TODO: What do we do with LQR stuff?
                ros::ServiceServer lqr_service;

                /** @brief Maps planning node name to planning query publisher */
                util::hash_t< std::string, ros::Publisher, util::string_hash> planning_node_to_queries_topic;

                /** @brief Maps planning node names to plan subscriptions */
                util::hash_t< std::string, ros::Subscriber, util::string_hash> plans_subscription;

                /** @brief Storage member for plant point*/
                util::space_point_t* plant_point;

                /** @brief Storage member for plant space*/
                const util::space_t* plant_space;

              public:

                planning_comm_t();
                ~planning_comm_t();


                //            bool listen_callback(listen_srv::Request& request, listen_srv::Response& response);

                /**
                 * Passes a \ref plan_t constructed from a plan message to a system.
                 * 
                 * @brief Passes a \ref plan_t constructed from a plan message to a system
                 * @param system_name The name of the system to pass the plan to
                 * @param plan The plan message
                 * @param e_state The end state of the plan (if any)
                 */
                void set_plan(const std::string& system_name, const std::vector<prx_simulation::control_msg>& plan, const std::vector<double>& e_state);

                // TODO: What to do we do with LQR stuff?
                void set_lqr(const std::string& controller_name, const std::vector<prx_simulation::lqr_msg>& all_lqrs);

                /**
                 * This function publishes a planning query, which a planning node will fulfill
                 * at some point.  It will return a plan which will fulfill the conditions
                 * specified in the query message. The plan can be received in the plans subscription.
                 * 
                 * @brief Publishes a new planning query
                 * @param start_state The start state of the planning query.
                 * @param goal_state The goal state of the planning query.
                 * @param goal_radius The acceptable distance to the goal.
                 * @param consumer_path The consumer requesting the planning query.
                 * @param planning_node The planning node name fulfilling the query.
                 * @param send_ground_truth Determines if ground truth needs to be sent as well.
                 * @param smooth Determines if the plan returned should be smoothed.
                 * @param set_goal_criterion Determines if the goal criterion will be set and used.
                 * @param homogeneous_setup Determines if the simulator has homogeneous setups for plants 
                 * @param duration How long to wait for service existence.
                 */
                void publish_planning_query(const std::vector<double>& start_state, const std::vector<double>& goal_state,
                                            double goal_radius, const std::string& consumer_path, const std::string& planning_node, bool send_ground_truth,
                                            bool smooth, bool set_goal_criterion, bool homogeneous_setup, double duration = 5);

                /**
                 * Creates a new planning node subscription to listen for plans
                 * 
                 * @brief Creates a new planning node subscription to listen for plans
                 * @param planning_node The name of the planning node to listen to
                 */
                void create_planning_subscription(const std::string& planning_node);

                /**
                 * Creates a new planning query topic.
                 * 
                 * @brief Creates a new planning query topic.
                 * @param planning_node The name of the planning node that will be queried to.
                 */
                void create_new_query_topic(const std::string& planning_node);

              protected:

                // TODO: What do we do with LQR stuff?
                bool lqr_callback(send_lqr_srv::Request& request, send_lqr_srv::Response& response);

                /**
                 * Callback which calls \ref set_plan()
                 * 
                 * @brief Callback that occurs when a plan is received from planning
                 * @param msg The received plan
                 */
                void plans_callback(const prx_simulation::plan_msg& msg);
            };

            extern sim_base_communication_t* plan_comm;

        }




    }
}

#endif

