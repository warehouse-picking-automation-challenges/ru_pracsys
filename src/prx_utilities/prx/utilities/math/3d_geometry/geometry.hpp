/**
 * @file geometry.hpp
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

#ifndef PRACSYS_GEOMETRY_HPP
#define	PRACSYS_GEOMETRY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/math/3d_geometry/trimesh.hpp"
#include "prx/utilities/boost/hash.hpp"


namespace prx
{
    namespace util
    {

        class parameter_reader_t;

        /** @brief Defines the geometry types in PRACSYS*/
        enum geometry_type
        {
            PRX_NONE = 0,
            PRX_SPHERE = 1,
            PRX_BOX = 2,
            PRX_CONE = 3,
            PRX_CYLINDER = 4,
            PRX_OPEN_CYLINDER = 5,
            PRX_CAPSULE = 6,
            PRX_TRIANGLE = 7,
            PRX_QUAD = 8,
            PRX_LINES = 9,
            PRX_LINESTRIP = 10,
            PRX_MESH = 11,
            PRX_CLOUD = 12,
            PRX_HEIGHTMAP = 13,
            PRX_POLYGON = 14
        };

        /**
         * This class contains all 3D geometries used in PRACSYS. It is responsible for
         * generating the corresponding trimesh (based on geometry type) as well as setting
         * the parameters associated with the geometry type
         *
         * @brief <b> 3D Geometry Class </b>
         *
         * @authors Andrew Kimmel, Athanasios Krontiris
         */
        class geometry_t
        {
        private:
            /** @brief the type of the geometry*/
            geometry_type type;
            /** @brief the scale and color of the geometry */
            vector_t scale, color;
            /** @brief the parameters of the geometry (i.e. radius, etc.) */
            std::vector<double> info;
            /** @brief the trimesh of the geometry */
            trimesh_t trimesh;
            /** @brief the filename of the mesh used to load the geometry */
            std::string mesh_filename;

        public:
            geometry_t();
            /**
             * Parameterized constructor that creates a geometry with the the params
             *
             * @brief Parameterized constructor that creates a geometry with the the params
             * @param type The type of geometry
             * @param params The parameters associated with the type of geometry
             */
            geometry_t(geometry_type type, const std::vector<double>* params);

            geometry_t(const geometry_t& geom);

            /**
             * Initializes a new geometry using a parameter reader
             *
             * @brief Initializes the geometry using a parameter reader
             * @param reader The parameter reader
             */
            void init(const parameter_reader_t* const reader);

            /**
             * Initializes a new geometry with geometry_type \c type and parameters \c dims
             *
             * @brief Initializes a new geometry with geometry_type \c type
             * @param type Determines the type of the geometry to be initialized
             * @param dims Contains initialization information (dimensions of geoms)
             *
             */
            void init_geometry(std::string type, vector_t& dims);

            /**
             * Sets the three dimensional scale of the geometry
             *
             * @brief Sets the three dimensional scale of the geometry
             * @param x The scale of X-dimension
             * @param y The scale of Y-dimension
             * @param z The scale of Z-dimension
             */
            void set_scale(double x, double y, double z);
            /**
             * Sets the color of the geometry
             *
             * @brief Sets the color of the geometry
             * @param r The red component of the color
             * @param g The green component of the color
             * @param b The blue component of the color
             * @param a The alpha component of the color
             */
            void set_color (double r, double g, double b, double a);

            /**
             * Specifically creates a sphere with radius \c radius
             *
             * @brief Creates a sphere
             * @param radius The radius of the sphere
             */
            void set_sphere(double radius);

            /**
             * Specifically creates a box with dimensions \c lx, \c ly, \c lz.
             * These are the lengths of the box.
             *
             * @brief Creates a box
             * @param lx length of x-dimension
             * @param ly length of y-dimension
             * @param lz length of z-dimension
             */
            void set_box(double lx, double ly, double lz);

            /**
             * Specifically creates a cone with radius \c radius and height \c height
             *
             * @brief Creates a cone
             * @param radius The radius of the cone
             * @param height The height of the cone
             */
            void set_cone(double radius, double height);

            /**
             * Specifically creates a cylinder with radius \c radius and height \c height
             *
             * @brief Creates a cylinder
             * @param radius The radius of the cylinder
             * @param height The height of the cylinder
             */
            void set_cylinder(double radius, double height);

            /**
             * Specifically creates an open cylinder with radius \c radius and height \c height
             *
             * @brief Creates an open cylinder
             * @param radius The radius of the open cylinder
             * @param height The height of the open cylinder
             */
            void set_open_cylinder( double radius, double height );

            /**
             * Specifically creates a capsule with radius \c radius and height \c height
             *
             * @brief Creates a capsule
             * @param radius The radius of the capsule
             * @param height The height of the capsule
             */
            void set_capsule( double radius, double height );

            /**
             * Specifically creates a triangle consisting of three vector_t's
             *
             * @brief Creates a triangle
             * @param v1 One of the lines of the triangle
             * @param v2 One of the lines of the triangle
             * @param v3 One of the lines of the triangle
             */
            void set_triangle( const vector_t* v1, const vector_t* v2, const vector_t* v3 );

            /**
             * Specifically creates a quadrilateral consisting of four vector_t's
             *
             * @brief Creates a quadrilateral
             * @param v1 One of the lines of the quadrilateral
             * @param v2 One of the lines of the quadrilateral
             * @param v3 One of the lines of the quadrilateral
             * @param v4 One of the lines of the quadrilateral
             */
            void set_quad( const vector_t* v1, const vector_t* v2, const vector_t* v3, const vector_t* v4);

            /**
             * Specifically creates multiple lines from a std::vector of endpoints defined by
             * vector_t's. This is NOT a linestrip.
             *
             * @brief Creates lines
             * @param points The points defining the lines
             */
            void set_lines( const std::vector<std::pair<vector_t, vector_t> >* points);

            /**
             * Specifically creates a linestrip from the vector_t's in \c points
             *
             * @brief Creates a linstrip
             * @param points Defines the points for the linestrip
             */
            void set_linestrip( const std::vector<vector_t>* points);

            /**
             * Creates a mesh geometry by loading a file
             *
             * @brief Loads a mesh
             * @param filename The name of the file to load the mesh
             */
            void set_mesh( const std::string& filename);

            /**
             * Creates a heightmap by loading a file
             *
             * @brief Loads a mesh
             * @param filename The name of the file to load the mesh
             */
            void set_heightmap( const std::string& filename);

            /**
             * Creates a polygon (polytope) from a series of triangles
             *
             * @brief Creates a polygon
             */
            void set_polygon( const std::vector<double>& triangles );


            /**
             * Creates a point cloud.
             *
             * @brief Creates a point cloud.
             */
            void set_point_cloud( );

            // Getters

            /**
             * Returns the type of the geometry
             *
             * @brief Returns the type of the geometry
             * @return The type of the geometry
             */
            geometry_type get_type() const;

            /**
             * Returns the parameters of the geometry
             *
             * @brief Returns the parameters of the geometry
             * @return The parameters of the geometry as a std::vector of doubles
             */
            const std::vector<double>* get_info() const;

            /**
             * Gets the radius of the sphere by reference
             *
             * @brief Gets the radius of the sphere
             * @param radius The radius of the sphere
             */
            void get_sphere(double& radius) const;

            /**
             * Gets the dimensions of the box by reference
             *
             * @brief Gets the dimensions of the box
             * @param lx The length of the box in the x-dimension
             * @param ly The length of the box in the y-dimension
             * @param lz The length of the box in the z-dimension
             */
            void get_box(double& lx, double& ly, double& lz) const;

            /**
             * Gets the dimensions of the cone by reference
             *
             * @brief Gets the dimensions of the cone
             * @param radius The radius of the cone
             * @param height The height of the cone
             */
            void get_cone(double& radius, double& height) const;

            /**
             * Gets the dimensions of the cylinder by reference
             *
             * @brief Gets the dimensions of the cylinder
             * @param radius The radius of the cylinder
             * @param height The height of the cylinder
             */
            void get_cylinder(double& radius, double& height) const;

            /**
             * Gets the dimensions of the open cylinder by reference
             *
             * @brief Gets the dimensions of the open cylinder
             * @param radius The radius of the open cylinder
             * @param height The height of the open cylinder
             */
            void get_open_cylinder( double& radius, double& height) const;

            /**
             * Gets the dimensions of the capsule by reference
             *
             * @brief Gets the dimensions of the capsule
             * @param radius The radius of the capsule
             * @param height The height of the capsule
             */
            void get_capsule( double& radius, double& height) const;

            /**
             * Gets the lines associated with the triangle by reference
             *
             * @brief Gets the dimensions of the triangle
             * @param v1 One line of the triangle
             * @param v2 One line of the triangle
             * @param v3 One line of the triangle
             */
            void get_triangle(vector_t* v1, vector_t* v2, vector_t* v3) const;

            /**
             * Gets the lines associated with the quadrilateral by reference
             *
             * @brief Gets the lines of the quadrilateral
             * @param v1 One line of the quadrilateral
             * @param v2 One line of the quadrilateral
             * @param v3 One line of the quadrilateral
             * @param v4 One line of the quadrilateral
             */
            void get_quad(vector_t* v1, vector_t* v2, vector_t* v3, vector_t* v4) const;

            /**
             * Gets the filename of the mesh
             *
             * @brief Gets the filename of the mesh
             * @return The filename of the mesh
             */
            const std::string& get_mesh() const;

            /**
             * Gets the filename of the heightmap
             *
             * @brief Gets the filename of the heightmap
             * @return The filename of the heightmap
             */
            const std::string& get_heightmap() const;

            /**
             * Gets the filename of the mesh
             *
             * @brief Gets the filename of the mesh
             * @return The filename of the mesh
             */
            const std::string& get_point_cloud_topic() const;

            /**
             * Gets the color of the geometry
             *
             * @brief Gets the color of the geometry
             * @return The color of the geometry
             */
            vector_t get_color() const;

            /**
             * Gets the scale of the geometry
             *
             * @brief Gets the scale of the geometry
             * @return The scale of the geometry
             */
            vector_t get_scale() const;

            /**
             * Gets the trimesh created for the geometry
             *
             * @brief Gets the trimesh created for the geometry
             * @param tri The trimesh of the geometry
             */
            void get_trimesh(trimesh_t* tri) const;

            /**
             * Gets the trimesh created for the geometry
             *
             * @brief Gets the trimesh created for the geometry
             * @return The trimesh of the geometry
             */
            const trimesh_t* get_trimesh() const;

            /**
             * Sets the parameters of the geometry
             *
             * @brief Sets the parameters of the geometry
             * @param params The parameters to set the geometry with
             */
            void set_params( const std::vector<double>* params );

            /**
             * This function is used to compute an approximate minkowsky sum.
             *
             * Currently, only boxes and cylinders are supported. Additionally, the object
             * added to the box or cylinder must be approximated by a bounding radius, which
             * is specified in \c object_radius.
             *
             * @brief Computes the approximate minkowski sum of the geometry
             * @param is_3D Determines if the computed minkowski sum will be 3D or 2D
             * @param object_radius The radius of the object added to the current geometry
             * @param epsilon The amount added to the sum
             * @return A pointer to a geometry which contains the approximate minkowski sum
             */
            geometry_t* get_minkowski(bool is_3D, double object_radius, double epsilon = 0.0) const;


            /**
             * Approximates the geometric object as a circle and returns
             * the corresponding bounding radius.
             *
             * @return The bounding radius for the geometric object
             */
            double get_bounding_radius() const;

            friend std::ostream& operator<<( std::ostream& out, const geometry_t& geom );
            
            bool contains_point(const vector_t& v, double buffer = 0);
        };

        /**
         * A structure that maps a unique rigid body name to its geometry.
         *
         * A rigid body name should be its system's name with a rigid body name.
         * For example: \c car1/chassis
         */
        typedef hash_t<std::string, geometry_t> geom_map_t;


    }
}

#endif





