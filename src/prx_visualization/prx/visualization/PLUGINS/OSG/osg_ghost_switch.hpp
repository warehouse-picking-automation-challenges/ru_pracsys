/**
 * @file osg_ghost_switch.hpp 
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


#ifndef PRX_OSG_GHOST_SWITCH_HPP
#define	PRX_OSG_GHOST_SWITCH_HPP

#include "prx/utilities/definitions/defs.hpp"

#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>

namespace prx 
{ 
    namespace vis 
    {

class osg_scene_t;

/**
 * This class handles switching between different modes of visualizing
 * OSG nodes.  It can switch between visualizing the collision geometries (ROUGH),
 * visualizing the models (FINE), or visualizing the models with the collision
 * geometries overlayed as ghosts (BOTH)
 * 
 * @brief <b> This class handles switching between different modes of visualizing OSG nodes. </b>
 *
 * @authors James Marble, Andrew Kimmel
 */
class osg_ghost_switch_t : public osgGA::GUIEventHandler
{

public:

    enum scene_mode_t { ROUGH = 0, FINE = 1, BOTH = 2} mode;
    const static int NUM_MODES = 3;

    osg_ghost_switch_t(osg_scene_t* scene) : mode(BOTH), scene(scene) {}

    /**
     * Processes switching visualization modes from keyboard input
     * 
     * @brief Processes switching visualization modes from keyboard input
     * @param event The UI event that occurred
     * @param action The actual action that occurred
     * @return True if the event was handled, false otherwise
     */
    bool handle( const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& action);
    
    /**
     * Overwrites the pick function to be empty
     * 
     * @brief Overwrites the pick function to be empty
     * @param view The current osgView
     * @param ea The event adapter
     */
    void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea) {}

    /**
     * Switches visualization modes
     * 
     * @brief Switches visualization modes
     * @param mode The mode to switch to
     */
    void switch_mode(scene_mode_t mode);


private:

    /**
     * @brief Implements the switch behavior
     */
    class switch_visitor_t : public osg::NodeVisitor
    {
    private:
        /** @brief The index of the child */
        int switch_child;

    public:
        switch_visitor_t(int switch_child) :
                osg::NodeVisitor::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
                switch_child(switch_child)
        {}

        /**
         * Applies the switch to the node
         * 
         * @brief Applies the switch to the node
         * @param node The node to apply the switch to
         */        
        void apply(osg::Switch& node)
        {
            PRX_DEBUG_S("Switching node " << node.getName() << " to geometry mode " << switch_child);
            node.setSingleChildOn(switch_child);
        }

    };

    /** @brief The scene to apply the switch to */
    osg_scene_t* scene;
};

    }
 }

#endif

