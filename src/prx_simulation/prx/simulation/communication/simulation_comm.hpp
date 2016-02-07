/**
 * @file simulation_comm.hpp 
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

#ifndef PRX_SIMULATION_SIMULATION_COMM_HPP
#define	PRX_SIMULATION_SIMULATION_COMM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/communication/sim_base_communication.hpp"

#include "prx_utilities/listen_srv.h"
#include "prx_simulation/state_msg.h"
#include "prx_simulation/request_ground_truth_srv.h"
#include "prx_simulation/request_space_information_srv.h"
#include "prx_simulation/set_sim_state_srv.h"

#include <std_msgs/Int32.h>

#include <ros/ros.h>

namespace prx
{
    namespace sim
    {

        using namespace prx_utilities;
        using namespace prx_simulation;

        class application_t;

        namespace comm
        {

            /**
             * @anchor simulation_comm_t
             *
             * This node is responsible for handling the simulation node's communication
             * to other nodes (publishing), as well as receiving any query messages 
             * from other nodes (subscribing).
             *
             * @brief <b> Planning's communication class for publishing and advertising.  </b>
             *
             * @authors Andrew Kimmel
             */
            class simulation_comm_t : public sim_base_communication_t
            {

              protected:

                // Advertised services
                //            ros::ServiceServer control_simulator_service;

                /** @brief Service which returns the ground truth of the simulation */
                ros::ServiceServer request_ground_truth_service;

                /** @brief Service which returns the space information of a particular system */
                ros::ServiceServer request_space_information_service;

                /** @brief Service which sets the simulation state*/
                ros::ServiceServer set_sim_state_service;

                /** @brief Topic which publishes ground truth messages */
                ros::Publisher ground_truth_topic;


                /** @brief Set of topics which handles updating specific planning nodes with their plant locations */
                util::hash_t< std::string, ros::Publisher, util::string_hash> planning_node_to_plant_locations_topic;

              public:
                simulation_comm_t();
                ~simulation_comm_t();

                //            void publish_state_spaces();

                // Planning ground truth

                /**
                 * This function publishes a ground truth message, which can be meant
                 * for a specific node.  The ground truth message is also time stamped.
                 * 
                 * @brief Publishes a time stamped ground truth message
                 * @param time The time stamp for the ground truth message
                 * @param node_name The name of the node the message is meant for (used to ignore certain messages)
                 * @param consumer_name Determines which consumer is publishing the ground truth. By default
                 *                      the entire ground truth is published (i.e. the simulator's entire state).
                 */
                void publish_ground_truth(double time, const std::string& node_name, const std::string& consumer_name = "ground_truth");


                //            void publish_system_states();

              protected:

                // TODO: Not implemented - do we need it?
                bool request_ground_truth_callback(request_ground_truth_srv::Request& request, request_ground_truth_srv::Response& response);

                bool set_sim_state_callback(set_sim_state_srv::Request& request, set_sim_state_srv::Response& response);

                /**
                 * This function is essential for planning to retrieve information and interpret
                 * ground truth messages from simulation.  It is used to inform planning how to correctly
                 * map its internal state to the ground truth state.
                 * 
                 * For example, let's say we have two 2D rigid body plants in the simulator.  Then
                 * the state space for simulation would look something like: [X,Y,THETA, X,Y,THETA].  
                 * Similarly, the control space would look like: [X,Y,THETA,X,Y,THETA]. Now let's say
                 * on our planning node, we are only responsible for planning for one of the plants.
                 * 
                 * Calling this service informs the planning node how to interpret ground truth messages
                 * (extract) so as to obtain the information relevant only to it (and thus not receive
                 * the extra state information). This is accomplished through the use of state intervals,
                 * which basically says "State X for Plant 2 is mapped at index 3", etc.
                 * 
                 * Similarly, this happens for the control space, and is used by planning to construct
                 * composite plans correctly (such as when planning for routers).
                 * 
                 * @brief Callback that occurs when space information is requested (see detailed doc)
                 * @param request The request of the service contains the consumer pathname 
                 * @param response The response of the service contains space intervals
                 * @return True if space information was retrieved
                 */
                bool request_space_information_callback(request_space_information_srv::Request& request, request_space_information_srv::Response& response);

            };

            extern sim_base_communication_t* sim_comm;


        }

    }
}

#endif

