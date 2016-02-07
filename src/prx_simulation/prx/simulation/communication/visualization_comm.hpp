/**
 * @file visualization_comm.hpp 
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

#ifndef PRX_SIMULATION_VISUALIZATION_COMM_HPP
#define	PRX_SIMULATION_VISUALIZATION_COMM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/communication/sim_base_communication.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/systems/system.hpp"
#include "prx/utilities/math/3d_geometry/geometry_info.hpp"

//#include "prx_utilities/describe_geometries_srv.h"
#include "prx_utilities/add_HUD_element_srv.h"
#include "prx_utilities/create_HUD_srv.h"
#include "prx_simulation/state_msg.h"


#include <std_msgs/Int32.h>

#include <ros/ros.h>


namespace prx
{
    namespace util
    {
        class tf_broadcaster_t;
    }

    namespace sim
    {

        using namespace prx_utilities;
        using namespace prx_simulation;

        class application_t;

        namespace comm
        {

            /**
             * @anchor visualization_comm_t
             *
             * This communication class is responsible for interfacing the
             * simulation node with the visualization node, from the simulation
             * node's side.
             *
             * @brief <b> Simulation node communication class for talking to visualization. </b>
             *
             * @authors Andrew Kimmel
             */
            class visualization_comm_t : public sim_base_communication_t
            {

              public:
                visualization_comm_t();


                /**
                 * This callback triggers when the listen service is called. The request
                 * of the service is the name of the node to listen to. This function will
                 * then subscribe to three topics under that node: keyboard, mouse selection,
                 * and system selection.  Typically this function is used to tie visualization
                 * node UI events to simulation.
                 * 
                 * @brief Callback that triggers when the listen service is called.
                 * @param request The request of the service
                 * @param response THe response of the service
                 * @return True
                 */
                bool listen_callback(listen_srv::Request& request, listen_srv::Response& response);

                /**
                 * This function initiates a service call to visualization, which in turn
                 * adds a new HUD space in the designated window.  The HUD space has either
                 * a background color or a texture, and is necessary in order to add HUD
                 * elements.
                 * 
                 * @brief Creates a HUD space in the visualization
                 * @param hud_name The name of the HUD
                 * @param location The window location of the HUD
                 * @param color The color of the HUD
                 * @param texture The name of the texture to use (if any)
                 * @return True
                 */
                bool create_hud(const std::string& hud_name, const std::vector<double>& location, const std::vector<double>& color, const std::string& texture);

                /**
                 * @brief Used to add HUD elements to a HUD space in visualization
                 * @param hud_name The name of the HUD space
                 * @param element_name The name of the HUD element
                 * @param color The color of the HUD element
                 * @param font The font for the HUD element (if it is text)
                 * @param char_size The font size for the HUD element (if it is text)
                 * @param position The position of the HUD element (relative to the HUD space)
                 * @param text The text of the HUD element
                 * @return True if the service call was successful, false otherwise
                 */
                bool update_hud_element(const std::string& hud_name, const std::string& element_name, const std::vector<double>& color, const std::string& font, const int char_size, const std::vector<double>& position, const std::string& text);

                /**
                 * @brief Send text string to visualize.
                 *
                 * Sends a string to the visualization to display.  The text will be
                 * anchored to a given system, and the font can be customized.
                 *
                 * @param anchored_system The system to which to attach the text string.
                 * @param text_name An identifier for this particular text.
                 * @param text The actual text to display on visualization.
                 * @param relative_position The position of the text relative to the anchored system.
                 * @param color The display color of the font.
                 * @param font_size The size of the font to display.
                 * @param font Which font to use to display the string.
                 */
                void visualize_scene_text(const std::string& anchored_system, const std::string& text_name, const std::string& text, const std::vector<double>& relative_position, const std::vector<double>& color, double font_size, const std::string& font);

                /**
                 * Uses the \c visualization_geom_map and the \c visualization_configs_map in order
                 * to send geometries and configurations to visualization.
                 * 
                 * @brief Used to send geometries to visualization
                 */
                void send_geometries();

                virtual bool send_extra_geometries(const util::hash_t<std::string, util::geometry_info_t>& new_geometries, double duration = 5, const std::string& destination_node = "visualization");

                /** @copydoc communication_t::update_info_geoms(const std::vector<std::string>& geom_names, const std::vector<util::config_t>& geom_configs, const std::vector<util::vector_t>& geom_colors, bool poll_tf, double duration = 5, const std::string& destination_node = "visualization") */
                virtual bool update_info_geoms(const std::vector<std::string>& geom_names, const std::vector<util::config_t>& geom_configs, const std::vector<util::vector_t>& geom_colors, bool poll_tf, double duration = 5, const std::string& destination_node = "visualization");

                virtual bool visualize_ghost_plant(const std::vector<std::string>& plant_paths, const std::vector<util::config_t>& plant_configs, double duration = 5, const std::string& destination_node = "visualization");

                virtual bool take_screenshot(unsigned screen_num, int number_of_screenshots, double duration = 5, const std::string& destination_node = "visualization");

                virtual bool visualize_obstacles(const std::string& path, double duration = 5, const std::string& destination_node = "visualization");
                
                //            // Publish specific batch configurations (information geometries)
                //            void publish_batch_configurations(const std::string& batch_name, std::vector< const util::config_t&>& config_msg);
                //
                //            // Publish new set of geometries
                //            void publish_geometries(const std::vector< util::geometry_info_t*>& geoms);
                //
                //            // Creates a new ros publisher
                //            void create_new_configuration_topic( std::string batch_name, std::string listen_node="simulation");

                /** @brief Maps a name to a util::geometry_info_t */
                util::hash_t<std::string, std::vector<util::geometry_info_t> > visualization_geom_map;

                /** @brief Maps a name to a util::config_t */
                util::hash_t<std::string, std::vector<util::config_t> > visualization_configs_map;
              protected:

                /**
                 * Informs the application which key button was pressed.
                 * 
                 * @brief Callback that occurs when keys have been pressed
                 * @param Contains the key that was pressed (as an integer)
                 */
                void update_pressed_keys_callback(const std_msgs::Int32& msg);

                /**
                 * Informs the application the 3D coordinates of the point clicked
                 * by the users in the scene (from visualization).
                 * 
                 * @brief Callback that occurs when users right click in the environment
                 * @param msg Contains the point that was clicked on
                 */
                void update_selected_point_callback(const geometry_msgs::Point& msg);

                /**
                 * Informs the application of the pathname of the selected system.
                 * 
                 * @brief Callback that occurs when users select a system with right click
                 * @param msg The pathname of the selected system
                 */
                void update_selected_callback(const std_msgs::String& msg);

                /**
                 * Informs the application of the new position of the camera.
                 * 
                 * @brief Callback that occurs when the camera moves
                 * @param msg The new position of the camera
                 */
                void update_camera_callback(const geometry_msgs::PoseArray& msg);

                /** @brief Publishes scene_text messages */
                ros::Publisher text_publisher;

                // Subscriptions (mainly to visualization)
                /** @brief Subscription to visualization for keyboard presses */
                ros::Subscriber keys_subscription;

                /** @brief Subscription to visualization for system selection */
                ros::Subscriber selected_system_subscription;

                /** @brief Subscription to visualization for point selection */
                ros::Subscriber selected_point_subscription;

                /** @brief Subscription to visualization for camera updates */
                ros::Subscriber camera_position_subscription;

                /** @brief Multiple subscriptions to planning nodes to receive plans */
                util::hash_t< std::string, ros::Subscriber, util::string_hash>plans_subscription;

                //            /** @brief Maps geometry batch names to their corresponding config topic*/
                //            util::hash_t< std::string, ros::Publisher, util::string_hash> batch_names_to_util::config_topics;

            };

            extern sim_base_communication_t* vis_comm;
            extern util::tf_broadcaster_t* tf_broadcaster;

        }
    }
}

#endif

