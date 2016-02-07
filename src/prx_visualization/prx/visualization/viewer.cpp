/**
 * @file viewer.cpp 
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

#include "prx/visualization/viewer.hpp"

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

/**
 * Viewer class constructor, which initializes the camera and handler as well as other
 * parameters.
 *
 * @param ihandler A pointer to the handler to attach to this viewer class.
 */
viewer_t::viewer_t( /*, simulator_t* simulator*/)
{
    handler = NULL;
}

viewer_t::~viewer_t()
{
    delete handler;
}

handler_t* viewer_t::get_handler()
{
    return handler;
}

void viewer_t::set_handler(handler_t* hand)
{
    if (handler != NULL)
    {
        delete handler;
        // TODO: Derived classes may need to detach this handler (OSG for example)
    }

    handler = hand;
}

void viewer_t::init(const parameter_reader_t* reader)
{
    full_screen_mode = reader->get_attribute_as<bool>("fullscreen");

    // RGBA
    back_color = reader->get_attribute_as<vector_t>("clear_color");
}

    }
 }
