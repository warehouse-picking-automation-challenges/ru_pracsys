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

#ifndef PRX_VISUALIZATION_COMMUNICATION_HPP
#define	PRX_VISUALIZATION_COMMUNICATION_HPP

#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"

#include "prx/visualization/scene_comm.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_window.hpp"

#include "prx_utilities/listen_srv.h"
#include "prx_utilities/create_HUD_srv.h"
#include "prx_utilities/add_HUD_element_srv.h"
#include "prx_utilities/add_twoD_text_srv.h"
#include "prx_utilities/update_twoD_text_srv.h"
#include "prx_utilities/update_info_geoms_srv.h"
#include "prx_utilities/Vec4_msg.h"
#include "prx_utilities/take_screenshot_srv.h"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace vis
    {

        class visualization_t;

        /**
         * This class is responsible for handling auxillary visualization
         * services - such as visualizing a HUD or communicating with a sensing node.
         * 
         * It is also responsible for starting the ROS services and topics that the
         * visualization node handles.
         * 
         * @brief <b> Handles auxiliary visualization services </b>
         *
         * @authors Andrew Kimmel
         */

        class visualization_comm_t : public scene_comm_t
        {

          private:

            // HUD stuff
            /** @brief add_HUD ROS service server */
            ros::ServiceServer add_HUD;
            /** @brief update_HUD ROS service server */
            ros::ServiceServer update_HUD;
            /** @brief Service used to update a set of info geoms */
            ros::ServiceServer update_info_geoms_service;
            /** @brief Service to allow a screenshot to be taken (from code) */
            ros::ServiceServer take_screenshot_service;
            /** @brief Maps camera names to active booleans */
            util::hash_t<std::string, bool> camera_sensors;
            /** @brief Used in callbacks to call visualization functions */
            visualization_t* vis;

          public:

            visualization_comm_t();

            void link_visualization(visualization_t* in_vis);

            /**
             * This callback creates a new HUD in the environment.
             * 
             * It asserts if \c vis is NULL.
             * 
             * @brief Callback for the service "create_HUD_srv"
             * @param request The service request 
             * @param response The service response
             * @return True if the callback was called
             */
            bool add_hud_callback(create_HUD_srv::Request& request,
                                  create_HUD_srv::Response& response);

            /**
             * 
             * @param request The service request
             * @param r The service response
             * @return True if the callback was called
             */
            bool update_hud_callback(add_HUD_element_srv::Request& request,
                                     add_HUD_element_srv::Response& r);
            
            /**
             * This callback allows for info geometry configurations to be updated
             * 
             * @brief Callback to quickly toggle the visualization of a plant
             * @param request The service request
             * @param response The service response
             * @return True if the callback is called
             */
            virtual bool update_info_geoms_callback(update_info_geoms_srv::Request& request,
                                                  update_info_geoms_srv::Response& response);
            
            /**
             * This callback allows for a screenshot to be taken programmatically 
             * 
             * @brief Callback for taking a screenshot
             * @param request The service request
             * @param response The service response
             * @return True if the callback is called
             */
            virtual bool take_screenshot_callback(take_screenshot_srv::Request& request,
                                                  take_screenshot_srv::Response& response);

            //    /**
            //     * 
            //     */
            //    void poll_topics(void);

            /**
             * It returns a pluginlib class loader for the current class.
             * 
             * @brief It returns a plugin lib class loader for the current class.
             * 
             * @return The pluginlib loader for the current class.
             */
            static pluginlib::ClassLoader<visualization_comm_t>& get_loader();


          private:


            /** 
             * The pluginlib loader for the \ref sim_base_communication_t class.
             * 
             * @brief The pluginlib loader for the \ref sim_base_communication_t class.
             */
            static pluginlib::ClassLoader<visualization_comm_t> loader;

        };

    }
}

#endif

