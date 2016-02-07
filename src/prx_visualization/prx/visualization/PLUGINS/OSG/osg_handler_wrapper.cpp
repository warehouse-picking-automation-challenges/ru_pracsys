/**
 * @file osg_handler_wrapper.cpp 
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


#include "prx/visualization/PLUGINS/OSG/osg_handler_wrapper.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_helpers.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_viewer.hpp"
#include "prx/visualization/visualization.hpp"

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

osg_handler_wrapper_t::osg_handler_wrapper_t(osg_viewer_t* osg_viewer, handler_t* const handler) :
    handler(handler), osg_viewer(osg_viewer)
{
}

osg_handler_wrapper_t::~osg_handler_wrapper_t()
{
}

bool osg_handler_wrapper_t::handle( const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& action)
{
    bool handled = false;
    vector_t camera_pos(3), eye_pos(3);;

    if (osg_viewer)
    {
        osg_viewer->get_windows()->at(0)->get_current_camera().get_camera_vector(PRX_CAMERA_CENTER, camera_pos);
        osg_viewer->get_windows()->at(0)->get_current_camera().get_camera_vector(PRX_CAMERA_EYE, eye_pos);
        handler->camera(camera_pos, eye_pos);
    }
    
    if (event.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
    {
        handler->keyboard( event.getKey() );
        handled = true;
    }
    else if (event.getEventType() == osgGA::GUIEventAdapter::PUSH
          && event.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
    {
        // -MUST- use a dynamic cast here!
        osgViewer::View* view = dynamic_cast<osgViewer::View*>(&action);
        if (view != NULL)
        {
            pick(view, event);
            handled = true;
        }
    }
    return handled;
}

/**
 * Helper function for determining if a node can be picked by the user.
 *
 * Special nodes such as the terrain and sky cannot be picked.
 *
 * @param node The node in question
 * @return true if the node is pickable
 */
bool pickable(const osg::Object* const object)
{
    const std::string& name = object->getName();
    return !name.empty()
            && name != "skydome" && name != "skydome_geode"
            && name != "terrain" && name != "Blender root";
}

void osg_handler_wrapper_t::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
    // A set of objects that will collide with our picker line segment.
    osgUtil::LineSegmentIntersector::Intersections intersections;

    // Query the scenegraph for collisions with the line segment extending
    // from the mouse pointer to the far-clipping plane.
    view->computeIntersections( ea.getX(), ea.getY(), intersections);

    std::string pickedName;
    PRX_DEBUG_S( " Before checking intersections");
    if( !intersections.empty() )
    {
        // Each intersection is actually a path from the root node all the
        // way to the geometry that was hit. We're not interested in the
        // geometry, however. We want the youngest osg::Group which
        // corresponds with an interesting object.
        const osg::NodePath path = intersections.begin()->nodePath;

        // If there are any special "unpickable" nodes in the path, don't bother.
        const osg::NodePath::const_iterator picked = std::find_if(path.begin(), path.end(), pickable);

        // if we haven't reached the end of our list of nodes
        if (picked != path.end())
        {
            pickedName = (*picked)->getName();
            PRX_INFO_S ("Picked name: " << pickedName);
            osg::Group* pickedGroup = checked_cast<osg::Group*>(*picked);
            osg_viewer->change_bounding_box(pickedGroup);
        }

    }
    if (!intersections.empty())
    {
        // call the intersect function with 2 rays
        vector_t ray = fromVec3(intersections.begin()->getWorldIntersectPoint());

        vector_t camera_pos(3);
        if (osg_viewer)
        {
            osg_viewer->get_windows()->at(0)->get_current_camera().get_camera_vector(PRX_CAMERA_CENTER, camera_pos);
            PRX_INFO_S("Picked point: " << ray);
            handler->point(ray);
        }
    }

    // if we did not pick soething valid
    if (!pickedName.empty())
    {   
        handler->pick(pickedName);
        PRX_INFO_S("Picked something with name " << pickedName);
    }
}

    }
 }
