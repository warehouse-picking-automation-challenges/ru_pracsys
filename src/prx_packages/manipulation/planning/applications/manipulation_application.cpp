/**
 * @file single_query.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/applications/manipulation_application.hpp"

#include "prx/planning/communication/planning_comm.hpp"
#include "prx/planning/communication/simulation_comm.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/task_planners/multi_planner/multi_planner_query.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"

#include "prx/utilities/statistics/statistics.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/communication/tf_broadcaster.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string.h>
#include <dirent.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_application_t, prx::plan::planning_application_t)


namespace prx
{
    using namespace util;
    using namespace plan;

    namespace plan
    {
        using namespace comm;
    }

    namespace packages
    {
        namespace manipulation
        {

            manipulation_application_t::manipulation_application_t()
            {
                //            interruptable = false;                
                serialize_flag = false;
                deserialize_flag = false;
                graph_builder = false;
                graph_user = false;
                ready_to_deserialize = false;
            }

            manipulation_application_t::~manipulation_application_t() { }

            void manipulation_application_t::init(const parameter_reader_t* reader)
            {

                PRX_DEBUG_COLOR("Initializing manipulation_application_t ...", PRX_TEXT_CYAN);
                single_query_application_t::init(reader);
                PRX_DEBUG_COLOR("Done initializing single_query_application_t ...", PRX_TEXT_GREEN);


                if( reader->has_attribute("graphs_filenames") )
                {
                    ungrasped_graph_file_name = reader->get_attribute("graphs_filenames/ungrasped_file", "blabla.txt");
                    grasped_graph_file_name = reader->get_attribute("graphs_filenames/grasped_file", "blublu.txt");
                }

                graph_builder = reader->get_attribute_as<bool>("graph_builder", false);
                graph_user = reader->get_attribute_as<bool>("graph_user", false);

                deserialize_flag = reader->get_attribute_as<bool>("deserialize", false);
                serialize_flag = reader->get_attribute_as<bool>("serialize", false);

                statistics_file_name = reader->get_attribute("statistics_file", ros::this_node::getName() + "_statistics.txt");

                if( graph_builder )
                {
                    serialize_flag = true;
                    root_task->set_param("", "graph_builder", graph_builder);
                    ((comm::planning_comm_t*)comm::plan_comm)->create_graph_publisher(ros::this_node::getName());
                    ((comm::planning_comm_t*)comm::plan_comm)->create_manip_ack_publisher(ros::this_node::getName());
                    ((comm::planning_comm_t*)comm::plan_comm)->subscribe_for_manip_ack(ros::this_node::getName(), reader->get_attribute("graph_user_node"));
                }
                else if( graph_user )
                {
                    deserialize_flag = true;
                    if( !reader->has_attribute("graph_builder_node") )
                        PRX_ERROR_S("Need to specify the graph builder node for this application!!!");
                    std::string graph_builder_node = reader->get_attribute("graph_builder_node");
                    ((comm::planning_comm_t*)comm::plan_comm)->subscribe_for_graphs(ros::this_node::getName(), graph_builder_node);
                    ((comm::planning_comm_t*)comm::plan_comm)->create_manip_ack_publisher(ros::this_node::getName());
                    ((comm::planning_comm_t*)comm::plan_comm)->subscribe_for_manip_ack(ros::this_node::getName(), graph_builder_node);
                }


                if( serialize_flag )
                {
                    root_task->set_param("", "serialize_flag", serialize_flag);
                    root_task->set_param("", "grasped_graph_file_name", grasped_graph_file_name);
                    root_task->set_param("", "ungrasped_graph_file_name", ungrasped_graph_file_name);
                }

                if( deserialize_flag )
                {
                    root_task->set_param("", "deserialize_flag", deserialize_flag);
                    root_task->set_param("", "grasped_graph_file_name", grasped_graph_file_name);
                    root_task->set_param("", "ungrasped_graph_file_name", ungrasped_graph_file_name);
                }

                char* w = std::getenv("PRACSYS_PATH");
                roadmaps_dir = std::string(w);
                roadmaps_dir += ("/prx_input/manipulation_roadmaps/");

                store_dir = std::string(w);
                //Is in prx_input folder because is from where the motion planner will read the graph.
                store_dir += ("/prx_output/manipulation_roadmaps/");

                if( graph_user )
                {
                    stats_file = std::string(w);
                    stats_file += ("/prx_output/statistics/" + statistics_file_name);

                    std::ofstream fout;
                    fout.open(stats_file.c_str());
                    fout << "graph_name" << "\t" <<
                            "start_time" << "\t" <<
                            "start_msg_time" << "\t" <<
                            "deserialize_time" << "\t" <<
                            "start_send_time" << "\t" <<
                            "sync_time" << "\t" <<
                            "receive_time" << "\t" <<
                            "serialize_time" << "\t" <<
                            "sending_time" <<
                            std::endl;
                    fout.close();
                }
                else
                {
                    file_index = 0;
                    list_file();
                }
            }

            void manipulation_application_t::execute()
            {

                //                PRX_DEBUG_POINT("Serialize: " << serialize_flag <<
                //                                "  Deserialize: " << deserialize_flag <<
                //                                "  grasped_file: " << grasped_graph_file_name <<
                //                                "  ungrapsed_file: " << ungrasped_graph_file_name <<
                //                                "  isBuilder:" << graph_builder <<
                //                                "  isUser: " << graph_user);

                if( graph_user )
                {
                    PRX_DEBUG_COLOR("WAIT for the graph to arrive...", PRX_TEXT_BROWN);
                    if( ready_to_deserialize )
                    {
                        PRX_DEBUG_COLOR("GOT the graph...", PRX_TEXT_CYAN);
                        single_query_application_t::execute();
                        //Time to restart the builder. 
                        ((comm::planning_comm_t*)comm::plan_comm)->publish_manip_ack(ros::this_node::getName(), 0, true);
                    }
                }
                else
                {
                    if( file_index >= file_names.size() )
                        return;

                    int start_time = get_time_in_sec();
                    ((comm::planning_comm_t*)comm::plan_comm)->publish_manip_ack(ros::this_node::getName(), start_time, true);

                    int start_msg_time = get_time_in_sec();
                    //serialization file is the file that the motion planner will 
                    //serialize the graph.We need to use this file to read and send the graph.
                    std::string file = roadmaps_dir + file_names[file_index];

                    PRX_DEBUG_COLOR("Will send the graph (" << file_index << ") : " << file_names[file_index], PRX_TEXT_CYAN);

                    statistics_clock.reset();
                    std::ifstream fin(file.c_str());
                    std::stringstream graph_string;
                    graph_string << fin.rdbuf();
                    fin.close();
                    double deserialize_time = statistics_clock.measure_reset();

                    ((comm::planning_comm_t*)comm::plan_comm)->publish_graph(ros::this_node::getName(), file_names[file_index], graph_string.str(), start_time, start_msg_time, deserialize_time, get_time_in_sec());
                }
            }

            std::pair<const std::string, const std::string> split_name(const std::string& name)
            {
                const size_t first_slash = name.find('_');

                if( first_slash == name.npos )
                    return std::make_pair(name, "");
                else
                    return std::make_pair(
                                          name.substr(0, first_slash),
                                          name.substr(first_slash + 1, name.npos));
            }

            void manipulation_application_t::process_graph_callback(const prx_simulation::graph_msg& msg)
            {
                int receive_time = get_time_in_sec();
                PRX_INFO_S("Graph_name is: " << msg.graph_name);

                statistics_clock.reset();
                std::string file = store_dir + msg.graph_name;
                std::ofstream fout;
                fout.open(file.c_str());
                fout << msg.graph;
                fout.close();
                double serialize_time = statistics_clock.measure_reset();

                fout.open(stats_file.c_str(), std::ofstream::app);
                fout.setf(std::ios::fixed, std::ios::floatfield);
                fout.precision(6);
                
                std::string graph_name = msg.graph_name;
                std::string type_str;
                std::string size_str;
                std::string number_str;
                std::string algo_str;
                
                boost::tie(type_str,graph_name) = split_name(graph_name);
                boost::tie(size_str,graph_name) = split_name(graph_name);
                boost::tie(number_str,algo_str) = split_name(graph_name);
                
                graph_name = algo_str + "_" + size_str + "_" + type_str + "_" + number_str;

                fout << graph_name << "\t," <<
                        msg.start_time << "\t," <<
                        msg.start_msg_time << "\t," <<
                        msg.deserialize_time << "\t," <<
                        msg.start_send_time << "\t," <<
                        sync_time << "\t," <<
                        receive_time << "\t," <<
                        serialize_time << "\t," <<
                        receive_time - msg.start_send_time <<
                        std::endl;
                fout.unsetf(std::ios::fixed);
                fout.close();

                //                ready_to_deserialize = true;
                ((comm::planning_comm_t*)comm::plan_comm)->publish_manip_ack(ros::this_node::getName(), 0, true);
            }

            void manipulation_application_t::process_ack_callback(const prx_simulation::manipulation_acknowledgement& msg)
            {
                if( graph_builder )
                {
                    file_index++;
                    PRX_DEBUG_COLOR(" I will restart here!!!", PRX_TEXT_RED);
                    execute();

                }
                else
                {

                    sync_time = msg.time - get_time_in_sec();
                    PRX_DEBUG_COLOR("Sync time : " << sync_time, PRX_TEXT_MAGENTA);
                }
            }

            int manipulation_application_t::get_time_in_sec()
            {
                time_t now = time(0);
                struct tm tstruct;
                tstruct = *localtime(&now);
                return tstruct.tm_hour * 3600 + tstruct.tm_min * 60 + tstruct.tm_sec;
            }

            void manipulation_application_t::list_file()
            {
                DIR *pDIR;
                struct dirent *entry;
                if( pDIR = opendir(roadmaps_dir.c_str()) )
                {
                    while( entry = readdir(pDIR) )
                    {
                        if( strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 )
                        {
                            file_names.push_back(std::string(entry->d_name));
                            PRX_DEBUG_COLOR(entry->d_name, PRX_TEXT_LIGHTGRAY);
                        }
                    }
                    closedir(pDIR);
                }
            }
        }
    }
}
