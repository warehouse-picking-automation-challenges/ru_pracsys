/**
 * @file osg_material.hpp 
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

#ifndef PRACSYS_OSG_MATERIAL_HPP
#define PRACSYS_OSG_MATERIAL_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_basic.hpp"

#include <vector>

namespace prx 
{ 
    namespace vis 
    {

/**
 * Wrapper around OSG state and material classes.  Materials allow objects
 * in the scene to have "color" (or more accurately, allow objects to have
 * a material attached to them).
 * 
 * @brief <b> Wrapper around OSG state and material classes </b>
 *
 * @authors Andrew Kimmel
 */
class osg_material_t : public osg::Material
{
public:
    osg_material_t();
    
    osg_material_t(const osg_material_t& other);
    
    osg_material_t& operator=(const osg_material_t & mat);

    /**
     * Initializes a material using the parameter reader. Requires ambient,
     * specular, shininess and diffuse values inside the reader.
     * 
     * @brief Initializes a material
     * @param reader Parameter reader
     */
    void init(const util::parameter_reader_t* reader);
    
    void set_all_modes();
    
    /** Data members */
public:
    
    osg::Material::Face diffuse_face; 
    osg::Vec4 diffuse_color;
    
    osg::Material::Face specular_face;
    osg::Vec4  specular_color;
    
    osg::Material::Face ambient_face; 
    osg::Vec4  ambient_color;
    
    osg::Material::Face shininess_face; 
    double  shininess_value;
    
    osg::Material::ColorMode color_mode;

};

    }
 }

#endif
