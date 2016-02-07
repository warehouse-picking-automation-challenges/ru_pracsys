/**
 * @file osg_hud.hpp 
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


#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_basic.hpp"

#ifndef PRACSYS_OSG_HUD_HPP
#define PRACSYS_OSG_HUD_HPP

namespace prx 
{ 
    namespace vis 
    {

/**
 * Container class for OSG HUD related objects.
 * 
 * @brief <b> Container class for OSG HUD related objects. </b>
 *
 * @authors Andrew Kimmel
 */
class osg_hud_t
{
public:
    osg_hud_t(osg::Geode*, osg::Projection*, osg::MatrixTransform*, osg::Geometry*, osg::Texture2D*);
    ~osg_hud_t();

    /** @brief The geode containing the HUD */
    osg::Geode* HUDGeode;
    /** @brief The projection matrix of the HUD */
    osg::Projection* HUDProjectionMatrix;
    /** @brief The matrix transform of the HUD */
    osg::MatrixTransform* HUDModelViewMatrix;
    /** @brief The background geometry of the HUD */
    osg::Geometry* HUDBackgroundGeometry;
    /** @brief The texture of the HUD*/
    osg::Texture2D* HUDTexture;
    /** @brief Maps HUD element names to their text objects */
    util::hash_t<std::string, osgText::Text*, util::string_hash> hud_element_map;

};

    }
 }

#endif	/* OSG_HUD_HPP */

