/**
 * @file ui_topic.hpp 
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


#ifndef PRX_UI_TOPIC_HPP
#define	PRX_UI_TOPIC_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/visualization/handler.hpp"


#include "prx/utilities/communication/communication.hpp"
#include <ros/ros.h>


namespace prx 
{ 
    namespace vis 
    {

/**
 * This class processes user inputs on the keyboard and mouse.
 * 
 * It advertises the input as specific ROS topics, for which other
 * nodes can listen for and process independently. Also advertises
 * updates on the camera's position.
 * 
 * @brief <b> Processes user inputs on the keyboard and mouse  </b>
 *
 * @authors Andrew Kimmel
 */
class ui_topic_t : public handler_t
{

private:

    /** @brief ROS node */
    ros::NodeHandle node;
    
    /** @brief Keyboard input topics */
    ros::Publisher key_topic;
    
    /** @brief Obtains a 3D point from right-mouse clicking topic */
    ros::Publisher point_topic;
    
    /** @brief Object selection with right-clicking*/
    ros::Publisher selected_topic;
    
    /** @brief Camera position topic*/
    ros::Publisher camera_topic;

public:

    ui_topic_t();

    /**
     * Publishes a message containing the pressed key
     * 
     * @brief Publishes a message containing the pressed key
     * @param input The integer value of the pressed key
     */
    void keyboard( int input );
    
    /**
     * Right-clicking in the scene without selecting an object
     * causes the corresponding right-clicked point 
     * to be generated and published here.
     * 
     * @brief Publishes a message containing the selected point
     * @param point The selected point
     */
    void point( const util::vector_t& point );

    /**
     * Right-clicking on an object in the scene causes it to be selected,
     * which will cause a message to be published containing the pathname of
     * the selected object.
     * 
     * @brief Publishes a message containing the name of the selected object
     * @param name The name/path of the selected object
     */
    void pick( const std::string& name );
    
    
    /**
     * Publishes a topic containing the current camera's current position
     * as well as where it is looking at.
     * 
     * @brief Publishes the position and the direction of the camera
     * @param camera The direction the camera is looking
     * @param eye The position of the camera 
     */
    void camera( const util::vector_t& camera, const util::vector_t& eye );
    
};

    }
 }

#endif

