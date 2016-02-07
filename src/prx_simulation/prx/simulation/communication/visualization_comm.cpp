/**
 * @file visualization_comm.cpp 
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

#include "prx/simulation/communication/visualization_comm.hpp"
#include "prx/simulation/applications/application.hpp"
#include "prx/utilities/communication/tf_broadcaster.hpp"


#include "prx/utilities/math/configurations/vector.hpp"
#include "prx_simulation/state_msg.h"

#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS(prx::sim::comm::visualization_comm_t, prx::sim::sim_base_communication_t)

namespace prx {
    using namespace util;

    namespace sim {
        namespace comm {

            sim_base_communication_t* vis_comm;
            tf_broadcaster_t* tf_broadcaster;

            /**
             *
             */
            visualization_comm_t::visualization_comm_t() {
                app = NULL;
                XmlRpc::XmlRpcValue sim_auto_send;
                node.getParam("sim_auto_send", sim_auto_send);
                if (sim_auto_send.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                    for (int32_t i = 0; i < sim_auto_send.size(); ++i) {
                        PRX_ASSERT(sim_auto_send[i].getType() == XmlRpc::XmlRpcValue::TypeString);
                        const std::string node_name = static_cast<std::string> (sim_auto_send[i]);
                    }
                } else {
                    PRX_WARN_S("No sim_auto_send parameter found, using default.");
                }


                //    default_config_topic = node.advertise<geometry_msgs::PoseArray > ("simulation/simulation_geometries/poses", 10);

                keys_subscription = node.subscribe("visualization/keys", 1, &visualization_comm_t::update_pressed_keys_callback, this);
                selected_point_subscription = node.subscribe("visualization/points", 60, &visualization_comm_t::update_selected_point_callback, this);
                selected_system_subscription = node.subscribe("visualization/selected", 60, &visualization_comm_t::update_selected_callback, this);
                camera_position_subscription = node.subscribe("visualization/camera", 60, &visualization_comm_t::update_camera_callback, this);
                text_publisher = node.advertise<prx_utilities::scene_text_msg > ("visualization/scene_text", 10);

            }

            /**
             *
             */
            void visualization_comm_t::update_pressed_keys_callback(const std_msgs::Int32& msg) {
                app->set_pressed_key(msg.data);
            }

            /**
             *
             */
            void visualization_comm_t::update_selected_point_callback(const geometry_msgs::Point& msg) {
                PRX_PRINT("We have a target at: " << msg.x << " , " << msg.y << " , " << msg.z, PRX_TEXT_GREEN);
                app->set_selected_point(msg.x, msg.y, msg.z);
            }

            /**
             *
             */
            void visualization_comm_t::update_selected_callback(const std_msgs::String& msg) {
                std::string path = msg.data;
                PRX_WARN_S("PATH TO SEARCH : " << path.c_str());
                app->set_selected_path(path);

            }

            /**
             *
             */
            void visualization_comm_t::update_camera_callback(const geometry_msgs::PoseArray& msg) {
                vector_t new_center(msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z);
                vector_t new_eye(msg.poses[1].position.x, msg.poses[1].position.y, msg.poses[1].position.z);

                app->set_camera(new_center, new_eye);

            }

            /**
             *
             */
            bool visualization_comm_t::listen_callback(listen_srv::Request& request, listen_srv::Response& response) {
                keys_subscription = node.subscribe(request.topic + "/keys", 60, &visualization_comm_t::update_pressed_keys_callback, this);

                selected_point_subscription = node.subscribe(request.topic + "/points", 60, &visualization_comm_t::update_selected_point_callback, this);

                selected_system_subscription = node.subscribe(request.topic + "/selected", 60, &visualization_comm_t::update_selected_callback, this);

                return true;
            }

            /**
             *
             */
            bool visualization_comm_t::create_hud(const std::string& hud_name, const std::vector<double>& location, const std::vector<double>& color, const std::string& texture) {
                if (app->is_visualizing()) {
                    ros::ServiceClient client = node.serviceClient<create_HUD_srv > ("visualization/add_HUD");
                    create_HUD_srv srv;

                    //  Actually add the request information
                    srv.request.hud_name = hud_name.c_str();
                    srv.request.area = location;
                    srv.request.color = color;
                    srv.request.texture = texture.c_str();

                    client.waitForExistence(ros::Duration(10));
                    if (!client.call(srv)) {
                        PRX_ERROR_S("Failed to send HUD to visualization!");
                        return false;
                    }
                    return true;
                } else {
                    PRX_WARN_S("Simulation is not visualizing!");
                    return false;
                }
            }

            /**
             *
             */
            bool visualization_comm_t::update_hud_element(const std::string& hud_name, const std::string& element_name,
                    const std::vector<double>& color, const std::string& font, const int char_size, const std::vector<double>& position, const std::string& text) {
                if (app->is_visualizing()) {
                    ros::ServiceClient client = node.serviceClient<add_HUD_element_srv > ("visualization/update_HUD");
                    add_HUD_element_srv srv;

                    //  Actually add the request information
                    srv.request.parent_name = hud_name.c_str();
                    srv.request.our_name = element_name.c_str();
                    srv.request.position = position;
                    srv.request.color = color;
                    srv.request.character_size = char_size;
                    srv.request.font = font.c_str();
                    srv.request.text = text.c_str();

                    client.waitForExistence(ros::Duration(10));
                    if (!client.call(srv)) {
                        PRX_ERROR_S("Failed to send HUD element to visualization!");
                        return false;
                    }
                    return true;
                } else {
                    PRX_WARN_S("Simulation is not visualizing!");
                    return false;
                }
            }

            void visualization_comm_t::visualize_scene_text(const std::string& anchored_system, const std::string& text_name, const std::string& text, const std::vector<double>& relative_position, const std::vector<double>& color, double font_size, const std::string& font) {
                PRX_DEBUG_S("Publishing scene text");
                prx_utilities::scene_text_msg msg;
                msg.anchored_system = anchored_system;
                msg.text = text;
                msg.text_name = text_name;
                msg.relative_position = relative_position;
                msg.color_rgba = color;
                msg.font = font;
                msg.font_size = font_size;

                text_publisher.publish(msg);
            }

            ///**
            // *  Creates config topic with name "listen_node/batch_name/poses" 
            // */
            //void visualization_comm_t::create_new_configuration_topic(std::string batch_name, std::string listen_node)
            //{
            //    batch_names_to_config_topics[batch_name] = node.advertise<geometry_msgs::PoseArray > (listen_node + "/" + batch_name + "/poses", 10);
            //    PRX_LOG_WARNING("Creating new service with name: %s", (listen_node + "/" + batch_name + "/poses").c_str());
            //}
            //
            ///** 
            // *  Publishes batch configurations in config_msg
            // */
            //void visualization_comm_t::publish_batch_configurations(const std::string& batch_name, std::vector< const config_t&>& config_msg)
            //{
            //    batch_names_to_config_topics[batch_name].publish(batch_configs_to_poseArray(config_msg));
            //}
            //
            //void visualization_comm_t::publish_geometries(const std::vector< geometry_info_t*>& geoms) { }

            void visualization_comm_t::send_geometries() {
                if (app->is_visualizing()) {
                    //need to transform the hash of name to vectors of geom_info's to a single hash of names to geom_infos.
                    hash_t<std::string, geometry_info_t> geoms;

                    foreach(std::string name, visualization_geom_map | boost::adaptors::map_keys) {
                        unsigned size = visualization_geom_map[name].size();
                        for (unsigned i = 0; i < size; i++) {
                            //            PRX_WARN_S("Geom name: "<<visualization_geom_map[name][i].full_name);
                            //            PRX_DEBUG_S ("Name: " << name);
                            geoms[ visualization_geom_map[name][i].full_name ] = visualization_geom_map[name][i];
                            tf_broadcaster->queue_config(visualization_configs_map[name][i], visualization_geom_map[name][i].full_name);
                        }
                    }

                    this->send_extra_geometries(geoms, 5);
                    //    sleep(1);
                    //    tf_broadcaster->broadcast_configs();
                } else {
                    PRX_WARN_S("Simulation is not visualizing!");
                }

            }

            bool visualization_comm_t::send_extra_geometries(const hash_t<std::string, geometry_info_t>& new_geometries, double duration, const std::string& destination_node) {
                if (app->is_visualizing()) {
                    PRX_DEBUG_COLOR("Send extra geometries", PRX_TEXT_GREEN);
                    return communication_t::send_extra_geometries(new_geometries, duration, destination_node);
                } else {
                    PRX_WARN_S("Simulation is not visualizing!");
                    return false;
                }
            }

            bool visualization_comm_t::update_info_geoms(const std::vector<std::string>& geom_names, const std::vector<util::config_t>& geom_configs, const std::vector<util::vector_t>& geom_colors, bool poll_tf, double duration, const std::string& destination_node) {
                if (app->is_visualizing()) {
                    PRX_DEBUG_COLOR("Update info geoms", PRX_TEXT_GREEN);
                    return communication_t::update_info_geoms(geom_names, geom_configs, geom_colors, poll_tf, duration, destination_node);
                } else {
                    PRX_WARN_S("Simulation is not visualizing!");
                    return false;
                }
            }

            bool visualization_comm_t::visualize_ghost_plant(const std::vector<std::string>& plant_paths, const std::vector<util::config_t>& plant_configs, double duration, const std::string& destination_node) {
                if (app->is_visualizing()) {
                    PRX_DEBUG_COLOR("Visualize ghosts", PRX_TEXT_GREEN);
                    return communication_t::visualize_ghost_plant(plant_paths, plant_configs, duration, destination_node);
                } else {
                    PRX_WARN_S("Simulation is not visualizing!");
                    return false;
                }

            }

            bool visualization_comm_t::take_screenshot(unsigned screen_num, int number_of_screenshots, double duration, const std::string& destination_node) {
                if (app->is_visualizing()) {
                    return communication_t::take_screenshot(screen_num, number_of_screenshots, duration, destination_node);
                } else {
                    PRX_WARN_S("Simulation is not visualizing!");
                    return false;
                }
            }

            bool visualization_comm_t::visualize_obstacles(const std::string& path, double duration, const std::string& destination_node) {
                if (app->is_visualizing()) {
                    PRX_DEBUG_COLOR("Visualize obstacles", PRX_TEXT_CYAN);
                    return communication_t::visualize_obstacles(path, duration, destination_node);
                } else {
                    PRX_WARN_S("Simulation is not visualizing!");
                    return false;
                }

            }
        }
    }
}
