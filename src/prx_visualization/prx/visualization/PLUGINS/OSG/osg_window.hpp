/**
 * @file osg_window.hpp 
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

#ifndef PRX_OSG_WINDOW_HPP
#define	PRX_OSG_WINDOW_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_basic.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_camera.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_scene.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx 
{ 
    namespace vis 
    {

class osg_viewer_t;

/**
 * This class creates a window for the visualization using OSG.
 * 
 * An OSG view is associated for each window, along with a set of cameras.
 * 
 * This class also handles user interaction with the window. For example, keyboard
 * input is mapped to certain camera functions and OSG functions, while clicking
 * the mouse is mapped to camera movement and object selection.
 * 
 * @brief <b> Creates a window using OSG and handles UI interaction </b>
 *
 * @authors Andrew Kimmel
 */
class osg_window_t : public osgGA::GUIEventHandler
{
private:

    /** @brief OpenSceneGraph view corresponding to the view this window has */
    osg::ref_ptr<osgViewer::View> view;
    /** @brief The set of cameras tied to this window */
    std::vector<osg_camera_t> cameras;
    /** @brief The coordinates of the mouse*/
    double mouse_x, mouse_y;
    /** @brief Set when the user left clicks */
    bool left_button;
    /** @brief Determines if the mouse cursor is visible and/or custom */
    bool mouse_visible, different_cursor;
    /** @brief The filename of the custom mouse cursor*/
    std::string cursor_filename;

    /** @brief Custom mouse pointer */
    osg::ref_ptr< osg::PositionAttitudeTransform > custom_mouse;

    /** @brief The camera light */
    osg::ref_ptr< osg::Light > camera_light;
    /** @brief The active camera */
    unsigned int active_camera;
    /** @brief The aspect ratio of the window*/
    double aspect_ratio;


public:

    osg_window_t(osg_viewer_t* osg_viewer);
    ~osg_window_t();

    /**
     * Retrieves the pointer to the view
     * 
     * @brief Retrieves the pointer to the view
     * @return The pointer to the view
     */
    osg::ref_ptr<osgViewer::View> get_wrapped_view() const;
    
    /**
     * Retrieves the current camera, which is defined by
     * the user. Cameras can be switched using the keyboard, F1 through F6
     * 
     * @brief Retrieves the current camera
     * @return The current camera
     */
    osg_camera_t& get_current_camera();

    /**
     * Calls frame on the current camera
     * 
     * @brief Calls frame on the current camera
     * @param pickedGroup The selected object (can be NULL)
     */
    void frame(osg::Group* pickedGroup);

    /**
     * Uses OSG classes to handle user interaction. Keyboard presses are
     * checked in a switch, and any mapped functions (such as moving the camera)
     * is then called.  This function handles any mouse clicking events as well.
     * 
     * @brief Handles user interaction with the window
     * @param event The type of event that occurred
     * @param action The type of action that occurred
     * @return Not used, except to match OSG function
     */
    bool handle( const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& action);

    /**
     * Updates the position of a custom mouse cursor
     * 
     * @brief Updates the position of a custom mouse cursor
     */
    void update_custom_mouse();

    /**
     * Initializes the window and cameras using OSG traits. For example,
     * the window size and position, the camera type and orientation, as well
     * the view is initialized in here.
     * 
     * @brief Initializes the window and cameras using OSG traits
     * @param reader Used to read in osg_window_t parameters
     * @param num The identifier for the window
     * @param back_color The background color
     * @param shared_context A pointer to the share context of this window (if any)
     * @param sceneData A pointer to the scene this window will visualize
     */
    void init(const util::parameter_reader_t* reader, int num, const util::vector_t& back_color,
            osg::GraphicsContext* shared_context, const osg_scene_t* sceneData);

private:

    /**
     * Sets the active camera
     * 
     * @brief Sets the active camera
     * @param num Determines which camera is set to active
     */
    void set_camera(const unsigned int num);

    osg_viewer_t* osg_viewer;
};

    }
 }

#endif
