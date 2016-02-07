/**
 * @file osg_viewer.hpp 
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

#ifndef PRACSYS_OSG_VIEWER_HPP
#define PRACSYS_OSG_VIEWER_HPP


#include "prx/visualization/viewer.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_camera.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_geode.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_window.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_texture.hpp"
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

namespace prx 
{ 
    namespace vis 
    {

/**
 * OSG implementation of the viewer_t abstract class 
 * 
 * Manages the viewer's scene, windows, and cameras
 * 
 * @brief <b> OSG implementation of the viewer_t abstract class </b>
 *
 * @authors Andrew Kimmel
 */
class osg_viewer_t : public viewer_t
{
private:

    /** @brief The rendered scene */
    osg_scene_t* osg_scene;

    /** @brief The windows belonging to the viewer*/
    std::vector< osg::ref_ptr<osg_window_t> > windows;

    /** @brief This handles main viewer and auxillary viewers */
    osgViewer::CompositeViewer master_viewer;

    /** @brief 6 cameras per viewer */
    std::vector<osg_camera_t> assigned_cameras;

    /** @brief Selected Group Node */
    osg::Group* selectedGroup;

    /** @brief The selection indicator bounding box if a selection was made */
    osg::ref_ptr<osg::Node> osg_bbox;

    /** @brief Used to determine when to update cameras */
    bool cameras_changed;

    /** @brief The skydome */
    osg::ref_ptr<osg::PositionAttitudeTransform> skydome;

    /** @brief Controls the orientation of the skydome */
    double sky_orientation;

    /** @brief Render ghosts flag */
    bool draw_ghosts;

    /** @brief Render the bounding box */
    bool render_bbox;
        
    /** @brief Maps window number to screenshot handler */
    std::vector< osg::ref_ptr<osgViewer::ScreenCaptureHandler> > screenshot_handlers;

public:
    osg_viewer_t();
    ~osg_viewer_t();

    /**
     * Sets up the sky dome, corresponding transformation
     * matrices, and rotation speed
     *
     * @brief Initializers sky dome.
     */
    void init_sky();
    
    /**
     * Sets up all the event handlers for the viewer specified in the handler
     * Responsible for creating and attaching handlers.
     *
     * @brief Initializes event handlers
     * @param num The view number for the handler
     */
    osg::ref_ptr<osgViewer::ScreenCaptureHandler> init_handlers( int num);

    /**
     * Indicates that a group is selected by adding a bounding box around it.
     *
     * If there already is a bounding box being displayed it is removed.
     *
     * @param selected_group Draw a bounding box around this group. If null,
     *                       then remove all bounding boxes.
     */
    void change_bounding_box( osg::Group* selected_group );
    
    void take_screenshot(unsigned screen_num, int num_screenshots);

    /**
     * @copydoc viewer_t::set_view_fullscreen()
     */
    void set_view_fullscreen();
    
    /**
     * @copydoc viewer_t::set_view_window()
     */
    void set_view_window( );
    
    /**
     * @copydoc viewer_t::set_clear_color()
     */
    void set_clear_color();

    /**
     * Check if the window is done visualizing.
     *
     * @brief Check if it is done.
     *
     * @remarks Substitution for run(), used with frame().
     *  while( !viewer.done() ) frame();
     *
     * @return Boolean if it is done.
     */
    bool done();
    
    /**
     * Step wise function where frame will be shown.
     *
     * @remarks Used in conjunction with done()
     *
     * @brief Show the frame.
     */
    void frame();
    
    /**
     * Run the Viewer to a loop where all the action and scene will be handled.
     *
     * @brief Run the Viewer.
     *
     * @remarks Any other control will not be supported
     * unless the window has been closed.
     */
    void run();

    /**
     * @copydoc viewer_t::get_scene()
     */
    osg_scene_t& get_scene();
    
    /**
     * @copydoc viewer_t::resize()
     */
    void resize();

    /**
     * Retrieves a std::vector of the viewer's windows
     * 
     * @brief Retrieves a std::vector of the viewer's windows
     * @return A vector of the viewer's windows
     */
    std::vector< osg::ref_ptr< osg_window_t> >* get_windows();

    /**
     * @copydoc viewer_t::set_handler()
     */
    void set_handler(handler_t* handler);

    /**
     * Toggles rendering ghosts
     * 
     * @brief Toggles rendering ghosts
     */
    void toggle_ghost();

    /**
     * Toggles rendering of the bounding box
     * 
     * @brief Toggles rendering of the bounding box
     */
    void toggle_bounding_box();

    /**
     * @copydoc viewer_t::init()
     * @note Initializes the scene, viewer and skydome
     */
    void init(const util::parameter_reader_t* reader);

};


    }
 }

#endif
