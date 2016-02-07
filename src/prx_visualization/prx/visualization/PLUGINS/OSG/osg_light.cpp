/**
 * @file osg_light.cpp 
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

#include "prx/visualization/PLUGINS/OSG/osg_light.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_helpers.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {
        
        osg_light_t::osg_light_t(unsigned int num)
        {
            light = new osg::Light(num);
            PRX_DEBUG_S("Creating light " << num);
        }
        
        void osg_light_t::set_pos(const vector_t &input)
        {
            PRX_ASSERT(input.get_dim() == 3);
            light->setPosition( osg::Vec4(input[0], input[1], input[2], 1) );
        }
        
        void osg_light_t::set_dir(const vector_t& dir)
        {
            light->setDirection( toVec3(dir) );
        }
        
        void osg_light_t::set_ambient(const vector_t& rgba)
        {
            light->setAmbient( toVec4(rgba) );
        }
        
        
        void osg_light_t::set_diffuse(const vector_t& rgba)
        {
            light->setDiffuse( toVec4(rgba) );
        }
        
        void osg_light_t::set_specular(const vector_t& rgba)
        {
            light->setSpecular( toVec4(rgba) );
        }
        
        
        void osg_light_t::set_spot_cutoff(double cutoff)
        {
            light->setSpotCutoff( cutoff );
        }
        
        
        osg::ref_ptr<osg::Light> osg_light_t::get_wrapped_light()
        {
            return light;
        }
        
        void osg_light_t::init(const parameter_reader_t* reader)
        {
            set_pos(reader->get_attribute_as<vector_t>("position"));
            set_dir(reader->get_attribute_as<vector_t>("direction"));
            set_ambient(reader->get_attribute_as<vector_t>("ambient"));
            set_diffuse(reader->get_attribute_as<vector_t>("diffuse"));
            set_specular(reader->get_attribute_as<vector_t>("specular"));
            
            light->setConstantAttenuation(.21);
            light->setQuadraticAttenuation(0.0);
            light->setLinearAttenuation(0.0);
            PRX_DEBUG_S("Light created");
        }
        
    }
}