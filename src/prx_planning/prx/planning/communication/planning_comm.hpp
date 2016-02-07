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

#ifndef PRX_PLANNING_PLANNING_COMM_HPP
#define	PRX_PLANNING_PLANNING_COMM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/communication/plan_base_communication.hpp"
#include "prx_utilities/query_planner_srv.h"
#include "prx_simulation/query_msg.h"
#include "prx_simulation/graph_msg.h"
#include "prx_simulation/manipulation_acknowledgement.h"
#include "prx_simulation/interval_msg.h"

#include "prx/simulation/plan.hpp"

#include <ros/ros.h>

namespace prx
{

    namespace util
    {
        class space_t;
    }

    namespace plan
    {

        class planning_application_t;

        using namespace prx_utilities;
        using namespace prx_simulation;

        namespace comm
        {

            /**
             * @anchor planning_comm_t
             * 
             * This node is responsible for handling the planning node's communication
             * to other nodes (publishing), as well as receiving any query messages 
             * from other nodes (subscribing).
             *
             * @brief <b> Planning's communication class for broadcasting information. </b>
             *
             * @author Andrew Kimmel
             */
            class planning_comm_t : public plan_base_communication_t
            {

              private:
                /** @brief Subscriber which subscribes to nodes which publish queries. */
                ros::Subscriber query_subscriber;
                /** @brief Publisher which specifically publishes poses. */
                ros::Publisher pose_publisher;
                /** @brief Publisher which reports plans computed by the planning node. */
                ros::Publisher plan_publisher;
                /** @brief Service which allows the planning node to be shutdown*/
                ros::ServiceServer shutdown_planning_service;

                /** @brief Subscriber which subscribes to nodes which publish graphs. */
                ros::Subscriber graph_subscriber;
                /** @brief Publisher which specifically publishes graphs. */
                ros::Publisher graph_publisher;

                util::hash_t<std::string, ros::Publisher, util::string_hash> ack_publishers;
                util::hash_t<std::string, ros::Subscriber, util::string_hash> ack_subscribers;

                /** @brief Maps planning node names to graph subscriptions */
                util::hash_t< std::string, ros::Subscriber, util::string_hash> graphs_subscription;

                std::string graph_publisher_name;


              public:
                planning_comm_t();

                virtual void link_application(planning_application_t* parent_app);
                /**
                 * @brief Request information about the space existing in the ground-truth simulator.
                 *
                 * In order to ensure a proper mapping between the simulation trees in 
                 * planning and in the ground truth simulation, this function requests
                 * the full space information from the simulator for the given consumer
                 * controller and stores state and control space information.
                 *
                 * @param consumer_path The path to the consumer for which we build the space info for.
                 * @param state_interval_message Returned state space interval information.
                 * @param control_interval_message Returned control space interval information.
                 */
                void request_space_information(const std::string& consumer_path, std::vector<interval_msg>& state_interval_msg, std::vector<interval_msg>& control_interval_msg);

                /**
                 * This callback occurs whenever a subscribed query topic publishes
                 * a query message.  This query message is passed to the application,
                 * which can then process it.
                 * 
                 * @brief Passes on the query message to the application for processing.
                 *
                 * @param msg The query message to be processed.
                 */
                void query_callback(const prx_simulation::query_msg& msg);

                /**
                 * Publishes computed plans for any node listening.
                 * 
                 * @brief Publishes computed plans for any node listening.
                 *
                 * @param system The system for which the plan is to be published for.
                 * @param plan The actual computed plan for the given system.
                 */
                void publish_plan(const std::string& system, const sim::plan_t& plan);

                bool shutdown_planning_callback(shutdown_node_srv::Request& request,
                                                shutdown_node_srv::Response& response);


                /*-------------------------------------------------------------*/
                /*-------------------------------------------------------------*/
                /*------------------  MANIPULATION MESSAGING ------------------*/
                /*-------------------------------------------------------------*/
                /*-------------------------------------------------------------*/
                /**
                 * This function will create a publisher for graphs given the name of the node, passed
                 * as argument. Any other node that needs to get the graphs has to subscribe on 
                 * this node. 
                 * 
                 * @param node_name
                 */
                void create_graph_publisher(const std::string& node_name);

                /**
                 * Publishes computed graphs in a string form. The receiver has to write the graph 
                 * in a file and deserialize it.
                 * @param graph_builder_planning_node The name of the node that send this message. 
                 * @param graph The graph in a string form. The argument graph is the string from the 
                 *              file that the graph serialized. 
                 */
                virtual void publish_graph(const std::string& graph_builder_planning_node, const std::string& graph_name, const std::string& graph_string, double start_time, double start_msg_time, double deserialize_time, double start_send_time);

                /**
                 * Subscribes to the planning_node node in order to read graph messages.
                 * 
                 * @param planning_node The node we want to subscribe. 
                 */
                virtual void subscribe_for_graphs(const std::string& planning_node, const std::string& graph_builder_planning_node);

                /**
                 * This callback occurs whenever a subscribed graph topic publishes
                 * a graph message.  This graph message is passed to the application,
                 * which can then process it.
                 * 
                 * @brief Passes on the graph message to the application for processing.
                 *
                 * @param msg The graph message to be processed.
                 */
                virtual void graph_callback(const prx_simulation::graph_msg& msg);
                
                /**
                 * This function will create a publisher acknowledgement for the manipulation problem. 
                 * The message contains the time that the graph builder start reading the graph and then 
                 * a boolean if the user has finish with the graph. This message will be send first to the 
                 * graph user from the graph builder in order to synchronize the time and then the user will
                 * send it back to terminate the builder. 
                 * 
                 * @param node_name
                 */
                void create_manip_ack_publisher(const std::string& node_name);

                /**
                 * Publishes computed graphs in a string form. The receiver has to write the graph 
                 * in a file and deserialize it.
                 * @param graph_builder_planning_node The name of the node that send this message. 
                 * @param graph The graph in a string form. The argument graph is the string from the 
                 *              file that the graph serialized. 
                 */
                virtual void publish_manip_ack(const std::string& publiser_node, double time, bool is_done);

                /**
                 * Subscribes to the planning_node node in order to read acknowlegement messages.
                 * 
                 * @param planning_node The node we want to subscribe. 
                 */
                virtual void subscribe_for_manip_ack(const std::string& planning_node, const std::string& publicer_planning_node);

                /**
                 * This callback occurs whenever a subscribed manipulation acknowledgement topic publishes
                 * an acknowledgement message. 
                 * 
                 * @brief Passes on the graph message to the application for processing.
                 *
                 * @param msg The graph message to be processed.
                 */
                virtual void manip_ack_callback(const prx_simulation::manipulation_acknowledgement& msg);
            };
            /** @brief External pointer for the single instance of the planning communication. */
            extern plan_base_communication_t* plan_comm;


        }

    }
}

#endif

