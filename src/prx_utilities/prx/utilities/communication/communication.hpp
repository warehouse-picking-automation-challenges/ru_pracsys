/**
 * @file communication.hpp 
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

#ifndef PRX_COMMUNICATION_HPP
#define PRX_COMMUNICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/3d_geometry/geometry_info.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/boost/hash.hpp"

#include "prx_utilities/listen_srv.h"
#include "prx_utilities/shutdown_node_srv.h"
#include "prx_utilities/send_plants_srv.h"
#include "prx_utilities/remove_plant_srv.h"
#include "prx_utilities/visualize_plant_srv.h"
#include "prx_utilities/visualize_obstacles_srv.h"
#include "prx_utilities/describe_geometries_srv.h"
#include "prx_utilities/update_info_geoms_srv.h"
#include "prx_utilities/visualize_ghost_plants_srv.h"
#include "prx_utilities/take_screenshot_srv.h"
#include "prx_utilities/query_planner_srv.h"


#include "prx_utilities/Vec4_msg.h"
#include "prx_utilities/rigid_body_info_msg.h"
#include "prx_utilities/scene_text_msg.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace prx
{
    namespace util
    {

        using namespace prx_utilities;

        /**
         * The default communication class which uses ROS to communicate
         * with other nodes. It contains common helper functions and common visualization functions 
         * which handles geometry messages for visualizing plants as well informational geometries.
         * @brief <b> Default communication class </b>
         *
         * @authors Andrew Kimmel
         */

        class communication_t
        {

          protected:

            /** @brief Each communication class comes with a ros::NodeHandle */
            ros::NodeHandle node;

            /** @brief Determines whether visualization will be checked for existence repeatedly */
            bool try_visualization_again;

            ros::Publisher vis_array_pub;
            visualization_msgs::MarkerArray array;

            // visualization_msgs::Marker send_marker(const geometry_info_t& info);
            // visualization_msgs::Marker send_marker(const geometry_t& info, std::string name);
            // visualization_msgs::Marker send_marker(const geometry_t& info, std::string name, const config_t& config);

          public:
            visualization_msgs::Marker send_marker(const geometry_info_t& info);
            visualization_msgs::Marker send_marker(const geometry_t& info, std::string name);
            visualization_msgs::Marker send_marker(const geometry_t& info, std::string name, const config_t& config);

            communication_t()
            {
                try_visualization_again = true;
                // vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
                vis_array_pub = node.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
            }

            virtual ~communication_t(){ }

            void publish_markers();

            void add_marker_to_array(const geometry_info_t& info)
            {
                array.markers.push_back(send_marker(info));
            }
            void add_marker_to_array(const geometry_t& info, std::string name)
            {
                array.markers.push_back(send_marker(info,name));
            }
            void add_marker_to_array(const geometry_t& info, std::string name,const config_t& config)
            {
                array.markers.push_back(send_marker(info,name,config));
            }

            /**
             * Helper function to convert a vector_t into a ROS::geometry_msgs::Point
             * 
             * @brief Converts a vector_t into a ROS::geometry_msgs::Point
             * @param point The vector_t to be converted
             * @return The converted vector_t as a ROS::geometry_msgs::Point
             */
            geometry_msgs::Point vector_t_to_point(const vector_t& point) const;

            /**
             * Helper function to convert a ROS::geometry_msgs::Point into a vector_t
             * 
             * @brief Converts a ROS::geometry_msgs::Point into a vector_t
             * @param point The ROS::geometry_msgs::Point to be converted
             * @return The converted ROS::geometry_msgs::Point as a vector_t
             */
            vector_t point_to_vector_t(const geometry_msgs::Point* point) const;

            /**
             * Helper function to convert a quaternion_t into a ROS::geometry_msgs::Quaternion
             * 
             * @brief Converts a quaternion_t into a ROS::geometry_msgs::Quaternion
             * @param quat The quaternion_t to be converted
             * @return The converted quaternion_t as a ROS::geometry_msgs::Quaternion
             */
            geometry_msgs::Quaternion quaternion_t_to_quaternion(const quaternion_t& quat) const;

            /**
             * Helper function to Convert a ROS::geometry_msgs::Quaternion into a quaternion_t
             * 
             * @brief Converts a ROS::geometry_msgs::Quaternion into a quaternion_t
             * @param quat The ROS::geometry_msgs::Quaternion to be converted
             * @return The converted ROS::geometry_msgs::Quaternion as a quaternion_t
             */
            quaternion_t quaternion_to_quaternion_t(const geometry_msgs::Quaternion* quat) const;


            /**
             * Helper function to convert a config_t into a ROS::geometry_msgs::Pose
             * 
             * @brief Converts a config_t into a ROS::geometry_msgs::Pose
             * @param config The config_t to be converted
             * @return The converted config_t as a ROS::geometry_msgs::Pose
             */
            geometry_msgs::Pose config_t_to_pose(const config_t& config) const;

            // TODO: Check later
            //    geometry_msgs::PoseArray config_ts_to_poseArray(const std::vector< const config_t&>& msg) const;
            //    geometry_msgs::PoseArray batch_configs_to_poseArray(const std::vector< const config_t&>& msg) const;

            /**
             * Helper function to convert a ROS::geometry_msgs::Pose into a config_t
             * 
             * @brief Converts a ROS::geometry_msgs::Pose into a config_t
             * @param pose The ROS::geometry_msgs::Pose to be converted
             * @return The converted ros::geometry_msgs::Pose as a config_t
             */
            config_t pose_to_config_t(geometry_msgs::Pose* pose) const;

            /**
             * Helper function to convert a ROS::geometry_msgs::PoseArrayConstPtr into a vector of config_t
             * 
             * @brief Converts a ROS::geometry_msgs::PoseArrayConstPtr into a vector of config_t
             * @param msg The geometry_msgs::PoseArray to be converted
             * @return The converted geometry_msgs::PoseArray as a vector of config_t
             */
            std::vector<config_t> poseArray_to_config_ts(const geometry_msgs::PoseArrayConstPtr msg) const;

            // TODO: Check later
            //    bool create_geometry_message(describe_geometries_srv::Request& request, std::vector<geometry_info_t>& info);

            /**
             * This function is used to send a map of geometries to another node. By default, this function calls a service to send a map of geometries
             * to the visualization node, which in turn visualizes the geometries.  These geometries are typically information geometries:
             * such as bounding lines, roadmaps, steering direction, etc. and is not meant to be used for physical geometries (such as
             * plants or obstacles).
             * 
             * @brief Used to visualize informational geometries (i.e. not physical plants or obstacles)
             * @param new_geometries A map of informational geometries indexed by name
             * @param duration How long ros should wait for existence of service
             * @param destination_node Where the geometries should be sent to
             * @return True if service call was successful, false otherwise
             */
            virtual bool send_extra_geometries(const hash_t<std::string, geometry_info_t>& new_geometries, double duration = 5, const std::string& destination_node = "visualization");

            /**
             * This function is used to add a multiple plants to the visualization node (or whatever destination node is specified).
             * This is accomplished through a service call to the destination node. Memory for the plant is assumed to be allocated then.
             * 
             * @param plant_paths The pathnames of the plants
             * @param template_paths The template paths, if they exist, for initializing plants using template parameter readers
             * @param duration How long ros should wait for existence of the service
             * @param destination_node Where the plants should be sent to
             * @return True if the service was successful, false otherwise
             */
            bool send_plants(const std::string& source_node, const std::vector<std::string>& plant_paths, const std::vector<std::string>& template_paths = std::vector<std::string>(), double duration = 5, const std::string& destination_node = "visualization");

            /**
             * This function is used to remove a single plant from the visualization node (or whatever destination node is specified).
             * This is accomplished through a service call to the dsetination node. Memory for the plant is assumed to be deallocated then.
             * 
             * @brief Used to remove a plant from visualization
             * @param path The pathname of the plant to be removed
             * @param duration How long ros should wait for existence of service
             * @param destination_node Where the plant has been visualized
             * @return True if service call was successful, false otherwise
             */
            bool remove_plant(const std::string& path, double duration = 5, const std::string& destination_node = "visualization");

            /**
             * This function is used to semi-frequently add or remove plants. It does not allocate or deallocate memory,
             * but rather toggles whether a plant is actively visualized (or not). This is accomplished through
             * a service call to the destsination node.
             * 
             * @brief Used to toggle the visualization of a plant (on/off)
             * @param path The pathname of the plant to be toggled
             * @param flag True to visualize the plant, false to not be visualized
             * @param duration How long ros should wait for existence of service
             * @param destination_node Where the plant exists
             * @return True if service call was successful, false otherwise
             */
            bool visualize_plant(const std::string& path, int flag, double duration = 5, const std::string& destination_node = "visualization");
            
            virtual bool visualize_obstacles(const std::string& path, double duration = 5, const std::string& destination_node = "visualization");
            
            virtual bool visualize_ghost_plant(const std::vector<std::string>& plant_paths, const std::vector<util::config_t>& plant_configs, double duration = 5, const std::string& destination_node = "visualization");
            virtual bool take_screenshot(unsigned screen_num, int number_of_screenshots, double duration = 5, const std::string& destination_node = "visualization");

            
            /**
             * @brief Handles updating a set of info geoms
             * @param geom_names The names of the info geoms to update
             * @param geom_configs The configurations to update the geoms with
             * @param poll_tf True: polls tf to obtain configs. False: Uses the configurations passed in as parameter.
             * @param duration How long ros should wait for existence of service
             * @param destination_node Where the plant exists
             */
            virtual bool update_info_geoms(const std::vector<std::string>& geom_names, const std::vector<util::config_t>& geom_configs, const std::vector<util::vector_t>& geom_colors, bool poll_tf, double duration = 5, const std::string& destination_node = "visualization");


            
            bool shutdown_node(const std::string& source_node, const std::string& destination_node, double duration = 5);


        };

    }
}

#endif


