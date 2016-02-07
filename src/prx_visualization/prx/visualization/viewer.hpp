#pragma once
/**
 * @file viewer.hpp 
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

#ifndef PRACSYS_VIEWER_HPP
#define PRACSYS_VIEWER_HPP


#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
//
#include "prx/visualization/handler.hpp"
#include "prx/visualization/camera.hpp"

namespace prx 
{ 
    namespace vis 
    {

/**
 * The abstract viewer class.  This class is responsible for providing
 * users with options to change their view of the scene (such as fullscreen
 * mode, orthographic, etc.)
 * 
 * @brief <b> Abstract viewer class  </b>
 *
 * @authors Andrew Kimmel
 */
class viewer_t
{
protected:
    /** @brief Handler for user interaction */
    handler_t* handler;                   
    
    /** @brief Whether the application is in full-screen mode*/
    bool full_screen_mode;
    
    /** @brief Whether the application is in orthographic mode*/
    bool ortho;
    
    /** @brief Vector representing the fill color for the visualization */
    util::vector_t back_color;
    
public:
    viewer_t( /*, simulator_t* simulator*/ );
    virtual ~viewer_t();

    /**
     * Used to set the application into full-screen mode
     * 
     * @brief Used to set the application into full-screen mode
     */
    virtual void set_view_fullscreen() = 0;
    
    /**
     * Used to set the position of the view window
     * 
     * @brief Used to set the position of the view window
     */
    virtual void set_view_window(  ) = 0;
    
    /**
     * Sets the fill color of the view window
     * 
     * @brief Sets the fill color of the view window
     */
    virtual void set_clear_color(  ) = 0;
    
    /**
     * Checks if the viewer is done visualizing
     * 
     * @brief Checks if the viewer is done visualizing
     * @return True if the viewer finished visualizing
     */
    virtual bool done()  = 0;
    
    /**
     * Any functions that need to happen with a certain framerate,
     * such as updating camera positions, will be called from this
     * function
     * 
     * @brief Runs a frame on the viewer
     */
    virtual void frame() = 0;
    
    /**
     * The viewer does not begin to run the frame function until
     * the run function has been called
     * 
     * @brief Starts framing the viewer
     */
    virtual void run()   = 0;

    /**
     * Resizes the viewer
     * 
     * @brief Resizes the viewer
     */
    virtual void resize() = 0;

    /**
     * Retrieves a pointer to the handler
     * 
     * @brief Retrieves a pointer to the handler
     * @return a pointer to the handler
     */
    virtual  handler_t* get_handler();
    
    /**
     * Sets the viewer's handler
     * 
     * @brief Sets the viewer's handler
     * @param handler Used to set the viewer's handler
     */
    virtual void set_handler(handler_t* handler);
    //virtual void get_windows();

    
    /**
     * Sets the full screen mode and clear screen color
     * 
     * @brief Initializes the viewer
     * @param reader Used to read in the viewer's attributes
     */
    void init(const util::parameter_reader_t* reader);

};

    }
 }

#endif
