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

#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/applications/planning_application.hpp"
#include "prx_simulation/plan_msg.h"
#include "prx_simulation/request_space_information_srv.h"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/spaces/embedded_space.hpp"

#include <geometry_msgs/PoseArray.h>

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

PLUGINLIB_EXPORT_CLASS(prx::plan::comm::planning_comm_t, prx::plan::plan_base_communication_t)


namespace prx
{
    using namespace util;
    using namespace sim;
    namespace plan
    {

        namespace comm
        {
            plan_base_communication_t* plan_comm;

            planning_comm_t::planning_comm_t()
            {

                planning_app = NULL;
            }

            void planning_comm_t::link_application(planning_application_t* parent_app)
            {
                PRX_INFO_S("Setting planning comm up with name: " << ros::this_node::getName());

                plan_base_communication_t::link_application(parent_app);

                std::string node_name = ros::this_node::getName();
                std::string sim_node = planning_app->get_simulation_node_name();
                pose_publisher = node.advertise<geometry_msgs::PoseArray > (node_name + "/poses", 10);
                plan_publisher = node.advertise<prx_simulation::plan_msg > (node_name + "/plans", 10, true);
                query_subscriber = node.subscribe(sim_node + node_name + "/planning_queries", 1, &planning_comm_t::query_callback, this);
                shutdown_planning_service = node.advertiseService(node_name + "/shutdown_node",
                                                                  &planning_comm_t::shutdown_planning_callback, this);

                PRX_WARN_S("Created query subscriber: " << sim_node + node_name + "/planning_queries");
            }

            void planning_comm_t::publish_plan(const std::string& system, const plan_t& plan)
            {
                sys_clock_t timer;
                timer.reset();
                //    PRX_INFO_S ("Publish plans in planning");

                prx_simulation::plan_msg new_plan;

                //    PRX_INFO_S("---Sending plans to " << system);

                new_plan.system_name = system;

                new_plan.plan.reserve(plan.size());

                foreach(plan_step_t step, plan)
                {
                    prx_simulation::control_msg control;
                    control.duration = step.duration;

                    //        PRX_WARN_S("publish duration: "<<control.duration);
                    control_t* storage;
                    if( dynamic_cast<embedded_point_t*>(step.control) != NULL )
                        storage = dynamic_cast<embedded_point_t*>(step.control)->link;
                    else
                        storage = step.control;
                    std::vector<double> converted_control = planning_app->plan_to_sim_control(storage);

                    for( unsigned int i = 0; i < converted_control.size(); ++i )
                        control.control.push_back(converted_control[i]);

                    new_plan.plan.push_back(control);
                }

                if( plan.get_end_state() != NULL )
                {
                    //        PRX_DEBUG_S("Plan end state is not null!");
                    for( unsigned i = 0; i < plan.get_end_state()->memory.size(); i++ )
                    {
                        new_plan.end_state.push_back(plan.get_end_state()->at(i));
                    }
                }

                if( plan_publisher.getNumSubscribers() == 0 )
                {
                    PRX_ERROR_S("No subscribers! Commence Sleeping");
                    double sleep_duration = 0;
                    while( plan_publisher.getNumSubscribers() == 0 )
                    {
                        sleep_duration += 0.1;
                        sleep(0.1);

                    }
                    PRX_WARN_S("*yawn* Good morning! Plan publisher slept for: " << sleep_duration);
                }
                plan_publisher.publish(new_plan);

                //    PRX_INFO_S ("Publish plans finished and took time: " << timer.measure());
            }

            void planning_comm_t::request_space_information(const std::string& consumer_path, std::vector<interval_msg>& state_interval_msg, std::vector<interval_msg>& control_interval_msg)
            {
                request_space_information_srv srv;


                // Construct request
                PRX_INFO_S("Requesting space information for consumer:  " << consumer_path);

                srv.request.consumer_pathname = consumer_path;
                std::string sim_node = planning_app->get_simulation_node_name();
                ros::ServiceClient client = node.serviceClient<request_space_information_srv > (sim_node + "/request_space_information");
                client.waitForExistence(ros::Duration(-1));
                if( !client.call(srv) )
                    PRX_FATAL_S("Failed to retrieve space information from simulation");

                // We've received a response...

                foreach(interval_msg msg, srv.response.state_space_intervals)
                {
                    state_interval_msg.push_back(msg);
                }

                foreach(interval_msg msg, srv.response.control_space_intervals)
                {
                    control_interval_msg.push_back(msg);
                }
            }

            void planning_comm_t::query_callback(const prx_simulation::query_msg& msg)
            {
                planning_app->process_query(msg);
            }

            bool planning_comm_t::shutdown_planning_callback(shutdown_node_srv::Request& request,
                                                             shutdown_node_srv::Response& response)
            {
                PRX_WARN_S("Received shutdown signal from node: " << request.source_node_name);
                ros::shutdown();
                return true;
            }

            void planning_comm_t::create_graph_publisher(const std::string& node_name)
            {
                graph_publisher = node.advertise<prx_simulation::graph_msg > (node_name + "/graphs", 10, true);
                graph_publisher_name = node_name;
            }

            void planning_comm_t::publish_graph(const std::string& graph_builder_planning_node, const std::string& graph_name, const std::string& graph_string, double start_time, double start_msg_time, double deserialize_time, double start_send_time)
            {
                prx_simulation::graph_msg msg;

                msg.sender_name = graph_builder_planning_node;
                msg.graph_name = graph_name;
                msg.graph = graph_string;
                msg.start_time = start_time;
                msg.start_msg_time = start_msg_time;
                msg.deserialize_time = deserialize_time;
                msg.start_send_time = start_send_time;

//                PRX_DEBUG_COLOR("Publish graph)  builder_node: " << graph_builder_planning_node << "   time: " << start_send_time, PRX_TEXT_BROWN);

                if( graph_publisher.getNumSubscribers() == 0 )
                {
                    double sleep_duration = 0;
                    while( graph_publisher.getNumSubscribers() == 0 )
                    {
                        sleep_duration += 0.1;
                        sleep(0.1);

                    }
                    PRX_WARN_S("Graph publisher slept for: " << sleep_duration);
                }
                graph_publisher.publish(msg);
            }

            void planning_comm_t::subscribe_for_graphs(const std::string& planning_node, const std::string& graph_builder_planning_node)
            {
                if( graphs_subscription.find(planning_node) == graphs_subscription.end() )
                {
                    PRX_DEBUG_COLOR("subscribe for the graphs at : " << graph_builder_planning_node << "/graphs", PRX_TEXT_BROWN);
                    graphs_subscription[planning_node] = node.subscribe(graph_builder_planning_node + "/graphs", 1, &planning_comm_t::graph_callback, this);
                }
            }

            void planning_comm_t::graph_callback(const prx_simulation::graph_msg& msg)
            {
                planning_app->process_graph_callback(msg);
            }

            void planning_comm_t::create_manip_ack_publisher(const std::string& node_name)
            {
                ack_publishers[node_name] = node.advertise<prx_simulation::manipulation_acknowledgement > (node_name + "/ack", 10, true);
            }

            void planning_comm_t::publish_manip_ack(const std::string& publiser_node, double time, bool is_done)
            {
                prx_simulation::manipulation_acknowledgement msg;

                msg.time = time;
                msg.done = is_done;

                if( ack_publishers[publiser_node].getNumSubscribers() == 0 )
                {
                    double sleep_duration = 0;
                    while( ack_publishers[publiser_node].getNumSubscribers() == 0 )
                    {
                        sleep_duration += 0.1;
                        sleep(0.1);

                    }
                    PRX_WARN_S("Manipulation acknowledgement publisher " << publiser_node << " slept for: " << sleep_duration);
                }
                ack_publishers[publiser_node].publish(msg);
            }

            void planning_comm_t::subscribe_for_manip_ack(const std::string& planning_node, const std::string& publicer_planning_node)
            {
                if( ack_subscribers.find(planning_node) == ack_subscribers.end() )
                {
                    ack_subscribers[planning_node] = node.subscribe(publicer_planning_node + "/ack", 1, &planning_comm_t::manip_ack_callback, this);
                }
            }

            void planning_comm_t::manip_ack_callback(const prx_simulation::manipulation_acknowledgement& msg)
            {
                planning_app->process_ack_callback(msg);
            }
        }
    }
}
