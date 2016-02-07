/**
 * @file osg_texture.cpp 
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

#include "prx/visualization/PLUGINS/OSG/osg_texture.hpp"

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

osg_texture_t::osg_texture_t()
{
}


bool osg_texture_t::has_terrain() const
{
    return is_terrain_alloc;
}


osg::ref_ptr<osg::Group> osg_texture_t::get_terrain() const
{
    
    return terrain;
}



void osg_texture_t::set_terrain_image(const std::string& path)
{
    terrain_image = osgDB::readImageFile(path);
    osg::ref_ptr<osg::ImageSequence> imageSequence = new osg::ImageSequence;
}


void osg_texture_t::get_discrete( double x, double y, double z, int divisor, bool is_terrain)
{
    double xStep = x/divisor, yStep = y/divisor;

    // Create a tile
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setName("terrain");

    // Set Vertices
    osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array;
    geom->setVertexArray( vert.get() );
    double 	xLeft = -x/2;
    double      xRight = -x/2 + xStep;
    double      yLeft = -y/2;
    double      yRight = -y/2 + yStep;
    vert->push_back(osg::Vec3( xRight, yRight, z));
    vert->push_back(osg::Vec3( xLeft,  yRight, z));
    vert->push_back(osg::Vec3( xLeft,  yLeft, z));
    vert->push_back(osg::Vec3( xRight, yLeft, z));

    // Set Normal
    osg::ref_ptr<osg::Vec3Array> norm = new osg::Vec3Array;
    norm->setName("terrain");
    geom->setNormalArray( norm.get() );
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    //if( is_terrain )
	norm->push_back(osg::Vec3(0.f, 0.f, 1.f));

    // Set Texture
    osg::ref_ptr<osg::Vec2Array> texture = new osg::Vec2Array;
    //Set Texture plane
    //if (is_terrain)
        geom->setTexCoordArray( terrain_num, texture.get() );
    texture->push_back(osg::Vec2(0.f, 0.f));
    texture->push_back(osg::Vec2(1.f, 0.f));
    texture->push_back(osg::Vec2(1.f, 1.f));
    texture->push_back(osg::Vec2(0.f, 1.f));

    // Set to 4-vertex quad & Add to Geode (Tile)
    geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4) );
    osg::ref_ptr<osg::Geode> tile = new osg::Geode;
    tile->setName("terrain");
    tile->addDrawable( geom.get() );
    tile->setDataVariance(osg::Object::DYNAMIC);

    // Create Terrain by using/translate Tile
   // if (is_terrain)
        terrain->setDataVariance( osg::Object::DYNAMIC );
    osg::Matrix m;
    for (int i=0; i<divisor; i++)
        for (int j=0; j<divisor; j++)
        {
            // Create a Transform Matrix Node
            osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform;
            trans->setDataVariance(osg::Object::STATIC);
            // Translate
            m.makeTranslate( j * xStep, i * yStep,  0.f );
            trans->setMatrix(m);
            trans->addChild( tile.get() );
            // Add it to Plane
           // if (is_terrain)
                terrain->addChild( trans.get() );
        }
    // Set Texture
    osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
    //Set plane Image
    tex->setImage( terrain_image.get() );
    tex->setUnRefImageDataAfterApply( true );
    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
    tex->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );

//    // Set State
   osg::StateSet* stateSet =  terrain->getOrCreateStateSet();
//    stateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
   stateSet->setTextureAttributeAndModes(terrain_num, tex);
//
//    // Enable depth test so that an opaque polygon will occlude a transparent one behind it.
//    stateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
//    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF);
//
//    // Conversely, disable writing to depth buffer so that
//    // a transparent polygon will allow polygons behind it to shine thru.
//    // OSG renders transparent polygons after opaque ones.
//    osg::Depth* depth = new osg::Depth;
//    depth->setWriteMask( false );
//    stateSet->setAttributeAndModes( depth, osg::StateAttribute::ON );

    // Disable conflicting modes.
    stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    osg::ref_ptr<osg::Material> mat = new osg::Material;
    if (!is_terrain)
        mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.8,0.8,0.8,0.15));
    else
        mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.8,0.8,0.8,1.0));

    // if night
   // mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.2,0.2,0.3,1.0));
    stateSet->setAttribute(mat);
}


void osg_texture_t::init(const parameter_reader_t* reader)
{
    terrain = NULL;
    terrain_num = 0;
    is_terrain_alloc = false;

    double att[3];
    int i_att;

    // Terrain Image
    terrain = new osg::Group();
    std::string filename = reader->get_attribute("terrain/terrain_image");
    PRX_INFO_S("Loading terrain file name " << filename);

    set_terrain_image( filename );
    if (reader->has_element("terrain/terrain_plane"))
    {
        PRX_DEBUG_S("Creating terrain plane.");
        att[0] = reader->get_attribute_as<double>("terrain/terrain_plane/x");
        att[1] = reader->get_attribute_as<double>("terrain/terrain_plane/y");
        att[2] = reader->get_attribute_as<double>("terrain/terrain_plane/z");
        i_att  = reader->get_attribute_as<double>("terrain/terrain_plane/divisor");

        is_terrain_alloc = true;
        is_selection_plane = false;
        get_discrete( att[0], att[1], att[2], i_att, true);
    }

    else if (reader->has_element("terrain/selection_plane"))
    {
        PRX_DEBUG_S("Creating selection plane.");
        att[0] = reader->get_attribute_as<double>("terrain/selection_plane/x");
        att[1] = reader->get_attribute_as<double>("terrain/selection_plane/y");
        att[2] = reader->get_attribute_as<double>("terrain/selection_plane/z");
        i_att  = reader->get_attribute_as<double>("terrain/selection_plane/divisor");

        is_terrain_alloc = true;
        is_selection_plane = true;
        get_discrete( att[0], att[1], att[2], i_att, false);
    }

}

    }
 }