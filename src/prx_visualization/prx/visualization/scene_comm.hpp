/**
 * @file scene_comm.hpp 
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

#ifndef PRX_SCENE_COMMUNICATION_HPP
#define	PRX_SCENE_COMMUNICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/communication/communication.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_scene.hpp"

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

namespace prx
{
    using namespace prx_utilities;
    
    namespace vis
    {

        /**
         * Communication class that handles direct visualization inside the scene.
         * Other nodes communicate with this class to visualize their objects.
         * This includes plants, obstacles, and scene text. 
         * 
         * Most of these operations occur through callbacks, which then call the appropriate
         * function in the scene class.
         * 
         * @brief <b> Communication class that handles directly visualizing objects in the scene </b>
         *
         * @authors Andrew Kimmel
         */
        class scene_comm_t : public util::communication_t
        {

          private:
            /** @brief A pointer to the scene */
            scene_t *scene;

          protected:

            /** @brief Service used to remove geometries from the scene */
            ros::ServiceServer remove_geometries_service;
            /** @brief Service used to add geometries from the scene */
            ros::ServiceServer decribe_geometries_service;
            /** @brief Service used to visualize plants in the scene */
            ros::ServiceServer send_plants_service;
            /** @brief Service used to remove plants from the scene */
            ros::ServiceServer remove_plant_service;
            /** @brief Service used to toggle visualization of plants in the scene */
            ros::ServiceServer visualize_plant_service;
            /** @brief Service used to visualize a set of obstacles in the scene */
            ros::ServiceServer visualize_obstacles_service;
            /** @brief Service used to visualize ghosts of plants in the scene */
            ros::ServiceServer visualize_ghost_plants_service;
            /** @brief Subscription to topics used to visualize text in the scene*/
            ros::Subscriber visualize_text_topic;


          protected:
            /** @brief A map from topic name to subscriptions */
            util::hash_t< std::string, ros::Subscriber, util::string_hash> subscriptions;

          public:

            scene_comm_t();

            /**
             * Links the scene to this communication class.
             * @return 
             */
            void link_scene(scene_t* in_scene);

            /**
             * This service callback processes the request, which contains a map of geometries
             * to be visualized. The scene extracts the information from the map and  visualizes
             * it in the scene.
             * 
             * @brief Callback to visualize geometries in the scene
             * @param request Contains a map of geometries to be visualized
             * @param response The service response (not used)
             * @return True if the callback is called
             */
            virtual bool geometry_callback(describe_geometries_srv::Request& request,
                                           describe_geometries_srv::Response& response);

            /**
             * This service callback processes the request, which contains the 
             * paths of the plants, and calls the scene to visualize the plants.
             * 
             * @brief Callback to visualize plants in the scene.
             * @param request Contains the paths of the plants
             * @param response The service response
             * @return True if the callback is called
             */
            virtual bool plants_callback(send_plants_srv::Request& request,
                                         send_plants_srv::Response& response);

            /**
             * This callback is used to remove the plant from the scene.
             * It is meant to be used infrequently, as it calls the scene to remove the plant
             * completely from the scene.
             * 
             * @brief Callback to remove plants from the scene
             * @param request The path of the plant to be removed
             * @param response The service response
             * @return True if the callback is called
             */
            virtual bool remove_plant_callback(remove_plant_srv::Request& request,
                                               remove_plant_srv::Response& response);

            /**
             * This callback toggles the visualization of plant. It is meant to be
             * used for semi-frequent calls, as the plant is not completely removed
             * from the scene.
             * 
             * @brief Callback to quickly toggle the visualization of a plant
             * @param request The service request
             * @param response The service response
             * @return True if the callback is called
             */
            virtual bool visualize_plant_callback(visualize_plant_srv::Request& request,
                                                  visualize_plant_srv::Response& response);
            /**
             */
            virtual bool visualize_obstacles_callback(visualize_obstacles_srv::Request& request,
                                                      visualize_obstacles_srv::Response& response);
            
            /**
             * This callback visualizes ghosts of plants.
             * 
             * @brief Ghosts.
             * @param request The service request
             * @param response The service response
             * @return True if the callback is called
             */
            virtual bool visualize_ghost_plants_callback(visualize_ghost_plants_srv::Request& request,
                                                  visualize_ghost_plants_srv::Response& response);

            /**
             * This callback constructs a scene_text instance from the message,
             * and subsequently calls the scene to visualize the text appropriately.
             * 
             * @brief Callback to visualize text in the scene
             * @param msg Message containing scene text information
             */
            virtual void text_callback(const prx_utilities::scene_text_msg& msg);

            /**
             * Retrieves a pointer to the scene
             * 
             * @brief Retrieves a pointer to the scene
             * @return A pointer to the scene
             */
            scene_t *get_scene(void) const
            {
                return scene;
            }
        };

    }
}

#endif	/* PRX_SCENE_COMMUNICATION_HPP */

