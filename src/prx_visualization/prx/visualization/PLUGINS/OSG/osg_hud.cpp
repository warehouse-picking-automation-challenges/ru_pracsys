/**
 * @file osg_hud.cpp 
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


#include "prx/visualization/PLUGINS/OSG/osg_hud.hpp"

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

osg_hud_t::osg_hud_t(osg::Geode* new_geode, osg::Projection* new_projection, osg::MatrixTransform* new_matrix, osg::Geometry* new_geom, osg::Texture2D* new_tex)
{
    HUDGeode = new_geode;
    HUDProjectionMatrix = new_projection;
    HUDModelViewMatrix = new_matrix;
    HUDBackgroundGeometry = new_geom;
    HUDTexture = new_tex;
}
osg_hud_t::~osg_hud_t()
{

}

//void osg_hud_t::update_hud(hud_element_t element)
//{
//    // give HUD the relevent information
//
//    // Check if the current hud element exists
//    if (hud_element_map.find(element.our_name)!= hud_element_map.end())
//    {
//        // Element exists, update data
//
//    }
//    // else
//}

    }
 }