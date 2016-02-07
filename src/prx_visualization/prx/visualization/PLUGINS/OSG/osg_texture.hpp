/**
 * @file osg_texture.hpp 
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

#ifndef PRACSYS_OSG_TEXTURE_HPP
#define PRACSYS_OSG_TEXTURE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_basic.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx 
{ 
    namespace vis 
    {

/**
 * OSG representation of a texture class.
 * 
 * Responsible for generating the ground terrain.
 * 
 * @brief <b> OSG representation of a texture </b>
 *
 * @authors Andrew Kimmel
 */
class osg_texture_t
{
private:
    /** @brief OSG group node to the terrain plane */
    osg::ref_ptr<osg::Group> terrain;
    /** @brief Image used to texture the terrain*/
    osg::ref_ptr<osg::Image> terrain_image;
    /** @brief Terrain identifier*/
    int terrain_num;
    /** @brief Flag for checking allocation of terrain*/
    bool is_terrain_alloc;

public:
    osg_texture_t();

    /**
     * Check if the texture has a terrain.
     *
     * @brief Check if Terrain exist.
     *
     * @return Boolean indicating whether the Terrain is allocated.
     */
    bool has_terrain() const;
    
    /**
     * Return the Terrain.
     *
     * @brief Get Terrain.
     *
     * @remarks This funciton is usually called by the Scene class.
     *
     * @return OSG node of the Terrain.
     */
    osg::ref_ptr<osg::Group> get_terrain() const;

    /**
     * Initialize the OSG Texture class.
     *
     * @brief Initialize.
     */
    void init(const util::parameter_reader_t* reader);

    /** @brief Marks if this texture is a selection plane*/
    bool is_selection_plane;

protected:

    /**
     * Sets the image that will be used for the Terrain.
     *
     * @brief Sets Terrain image.
     *
     * @remarks Tested image format: tga.
     *
     * @param file The filename of the image file.
     */
    void set_terrain_image( const std::string& file );
    
    /**
     * Create a diskretized Sky or Terrain given the value, divisor, and boolean.
     * Used for making the code efficient.
     *
     * @brief Sets diskretized Sky or Terrain.
     *
     * @remarks Default position is (0, 0, 0).
     *
     * @param x The distance value related to the x axis.
     * @param y The distance value related to the y axis.
     * @param z The distance value related to the z axis.
     * @param divisor The divisor value to create tiles of same images.
     * @param is_terrain The boolean value indicating if it is terrain.
     */
    void get_discrete(double x, double y, double z, int divisor, bool is_terrain);
};

    }
 }
#endif
