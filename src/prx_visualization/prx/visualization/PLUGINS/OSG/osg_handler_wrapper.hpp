/**
 * @file osg_handler_wrapper.hpp 
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


#ifndef PRX_OSG_HANDLER_WRAPPER_HPP
#define	PRX_OSG_HANDLER_WRAPPER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/visualization/handler.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_basic.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_viewer.hpp"

namespace prx 
{ 
    namespace vis 
    {

/**
 * Wraps a handler in an osgGA::GUIEventHandler so OSG events are passed
 * to the implementation of the handler.
 * 
 * @brief <b> Wraps a GUI Event handler </b>
 *
 * @authors Andrew Kimmel
 */
class osg_handler_wrapper_t : public osgGA::GUIEventHandler
{

public:
    osg_handler_wrapper_t(osg_viewer_t* osg_viewer, handler_t* const handler);
    ~osg_handler_wrapper_t();

    virtual bool handle( const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& action);
    virtual void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

private:
    handler_t* const handler;
    osg_viewer_t* osg_viewer;
};

    }
 }

#endif	/* PRX_OSG_HANDLER_WRAPPER_HPP */
