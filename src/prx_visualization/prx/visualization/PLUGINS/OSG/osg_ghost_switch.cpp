/**
 * @file osg_ghost_switch.cpp 
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


#include "prx/visualization/PLUGINS/OSG/osg_ghost_switch.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_scene.hpp"

#include "osg_visualization.hpp"

#include <osg/NodeVisitor>
#include <osg/Switch>

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

bool osg_ghost_switch_t::handle( const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& action)
{
    bool handled = false;

    if (event.getEventType() == osgGA::GUIEventAdapter::KEYDOWN
            && event.getKey() == 'g' )
    {
        switch_mode((scene_mode_t)(mode+1));
        handled = true;
    }

    return handled;
}

void osg_ghost_switch_t::switch_mode(scene_mode_t mode)
{
    // Cycle to the next mode.
    mode = (scene_mode_t)(mode % NUM_MODES);

    if(this->mode != mode)
    {
        PRX_DEBUG_S("Switching to geometry mode " << mode);

        osg::Node* root = scene->get_wrapped_root();

        switch_visitor_t visitor(mode);
        root->accept(static_cast<osg::NodeVisitor&>(visitor));
        
        this->mode = mode;
    }
}

    }
 }
