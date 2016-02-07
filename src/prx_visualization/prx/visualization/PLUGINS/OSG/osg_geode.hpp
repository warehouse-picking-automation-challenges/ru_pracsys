/**
 * @file osg_geode.hpp 
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

#ifndef PRX_OSG_GEODE_HPP
#define PRX_OSG_GEODE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/utilities/boost/hash.hpp"

#include <osg/Geode>
#include <osg/Node>
#include <osg/ref_ptr>

namespace prx 
{ 
    namespace vis 
    {

/**
 * Helper class which sets up osg::Geodes from util::geometry_t.
 * 
 * Also handles loading meshes from file, and creating an osg::Node for the mesh.
 * 
 * @brief <b> Helper class to create osg::Geodes from util::geometry_t </b>
 *
 * @authors Andrew Kimmel
 */
class osg_geode_t
{
public:
    /**
     * 
     * @brief Sets up an osg::Geode by using the information passed as a util::geometry_t
     * @param geometry Contains the information necessary to create an osg::Geode
     * @param transparent Whether or not the osg::Geode is transparent
     * @param line_thickness If the osg::Geode is a line (or line strip), determines the thickness
     * @return 
     */
    static osg::ref_ptr<osg::Geode> setup_geometry(const util::geometry_t* geometry, bool transparent, double line_thickness = 2.0);
    
    /**
     * Sets up a mesh and returns a pointer to the osg::Node. Checks if the
     * mesh has already been loaded - if it has been loaded, the \c mesh_lookup
     * is used for quick retrieval.
     * 
     * @brief Loads a mesh. Previously loaded meshes are accessed through the \c mesh_lookup
     * @param filename The filename of the mesh
     * @param mesh_lookup Maps the filename of the mesh to its corresponding osg::Node
     * @return A smart pointer to the loaded mesh
     */
    static osg::ref_ptr<osg::Node>  setup_mesh(const std::string& filename, util::hash_t< std::string, osg::ref_ptr<osg::Node>, util::string_hash>* mesh_lookup);
    
    /**
     * Loads a mesh
     * 
     * @brief Loads a mesh
     * @param filename The filename of the mesh
     * @return A smart pointer to the loaded mesh
     */
    static osg::ref_ptr<osg::Node>  setup_mesh(const std::string& filename);

private:
    // Can't construct an instance of this static-only helper class.
    osg_geode_t() {}
};

    }
 }
#endif
