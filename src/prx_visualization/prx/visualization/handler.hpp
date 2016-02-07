/**
 * @file handler.hpp 
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

#ifndef PRACSYS_HANDLER_HPP
#define	PRACSYS_HANDLER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"


namespace prx 
{ 
    namespace vis 
    {

/**
 * This class is the input handler for the visualization tools, handling keyboard input.
 *
 * @brief <b> Input Handler for the visualization </b>
 *
 * @author Andrew Kimmel
 */
class  handler_t
{
public:

    /**
     * The Abstract input handler destructor that does nothing
     */
    virtual ~handler_t(){};

    /**
     * Handles the interaction with the keyboard given some key as input
     *
     * @brief Handles the interaction with the keyboard given some key as input
     * @param input The key that was pressed on the keyboard encoded as an integer
     */
    virtual void keyboard( int input ) = 0;

    /**
     * Called when the user clicks on nothing pickable
     * 
     * @brief Called when the user clicks on nothing pickable
     * @param point The 3D coordinates of the point selected by the user.
     */
    virtual void point( const util::vector_t& point ) = 0;

    /**
     * Called when a body is picked by the user's mouse.
     *
     * @brief Called when a body is picked by the user's mouse.
     * @param name The name of the object selected by the user.
     */
    virtual void pick( const std::string& name ) = 0;

    /**
     * Called when a user moves the camera
     * 
     * @brief Called when a user moves the camera
     * @param camera The position of the camera
     * @param eye The direction the camera is looking
     */
    virtual void camera( const util::vector_t& camera, const util::vector_t& eye ) = 0;
}; 

    }
 }

#endif

