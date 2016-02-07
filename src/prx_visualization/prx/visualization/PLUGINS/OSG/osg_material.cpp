/**
 * @file osg_material.cpp 
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

#include "prx/visualization/PLUGINS/OSG/osg_material.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_helpers.hpp"

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

osg::Material::Face parseFace (const std::string& face)
{
    osg::Material::Face result;
    if ( face == "front" )
        result = osg::Material::FRONT;
    else if ( face == "back" )
        result = osg::Material::BACK;
    else if ( face == "front_back" )
        result = osg::Material::FRONT_AND_BACK;
    else
        PRX_FATAL_S("Invalid face mode: " << face);

    return result;
}

osg::Material::ColorMode parseColorMode(const std::string& mode)
{
    osg::Material::ColorMode result;

    if ( mode == "ambient" )
        result = osg::Material::AMBIENT;
    else if ( mode == "diffuse" )
        result = osg::Material::DIFFUSE;
    else if ( mode == "specular" )
        result = osg::Material::SPECULAR;
    else if ( mode == "emission" )
        result = osg::Material::EMISSION;
    else if ( mode == "ambient_diffuse" )
        result = osg::Material::AMBIENT_AND_DIFFUSE;
    else if ( mode == "off" )
        result = osg::Material::OFF;
    else
        PRX_FATAL_S("Invalid color mode: " << mode);

    return result;
}

osg_material_t::osg_material_t()
{
}

void osg_material_t::init(const parameter_reader_t* reader)
{
    PRX_DEBUG_S(" The diffuse face : " << reader->get_attribute("diffuse/faces"));

    diffuse_face = parseFace(reader->get_attribute("diffuse/faces"));
    diffuse_color = toVec4(reader->get_attribute_as<vector_t>("diffuse/color"));
    specular_face = parseFace(reader->get_attribute("specular/faces"));
    specular_color = toVec4(reader->get_attribute_as<vector_t>("specular/color"));
    ambient_face = parseFace(reader->get_attribute("ambient/faces"));
    ambient_color = toVec4(reader->get_attribute_as<vector_t>("ambient/color"));
    shininess_face = parseFace(reader->get_attribute("shininess/faces"));
    shininess_value = reader->get_attribute_as<double>("shininess/value");
    color_mode = parseColorMode(reader->get_attribute("color_mode"));

    set_all_modes();

}

void osg_material_t::set_all_modes()
{
    setDiffuse(diffuse_face, diffuse_color);
    setSpecular(specular_face, specular_color);
    setAmbient(ambient_face, ambient_color);
    setShininess(shininess_face, shininess_value);
    setColorMode(color_mode);
}

osg_material_t::osg_material_t( const osg_material_t & other ) : osg::Material()
{
    diffuse_face = other.diffuse_face;
    diffuse_color = other.diffuse_color;
    specular_face = other.specular_face;
    specular_color = other.specular_color;
    ambient_face = other.ambient_face;
    ambient_color = other.ambient_color;
    shininess_face = other.shininess_face;
    shininess_value = other.shininess_value;
    color_mode = other.color_mode;
    
    set_all_modes();
}

osg_material_t& osg_material_t::operator= ( const osg_material_t& mat )
{
    
    this->diffuse_face = mat.diffuse_face;
    this->diffuse_color = mat.diffuse_color;
    this->specular_face = mat.specular_face;
    this->specular_color = mat.specular_color;
    this->ambient_face = mat.ambient_face;
    this->ambient_color = mat.ambient_color;
    this->shininess_face = mat.shininess_face;
    this->shininess_value = mat.shininess_value;
    this->color_mode = mat.color_mode;
    
    this->set_all_modes();
    return *this;
}

    }
 }
