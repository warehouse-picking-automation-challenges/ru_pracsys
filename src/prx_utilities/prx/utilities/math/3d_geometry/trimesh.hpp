/**
 * @file trimesh.hpp
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

#ifndef PRACSYS_TRIMESH_HPP
#define PRACSYS_TRIMESH_HPP


#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include <vector>

namespace prx
{
    namespace util
    {

        /**
         * A struct that keeps the information for a face of the trimesh. Fase represent
         * each triangle that the trimesh is constructed with. Each face has 3 indices
         * for the 3 corners of the triangle.
         *
         * @brief <b> A struct that keeps the information for a face of the trimesh.</b>
         */
        typedef struct trimesh_face
        {
            int index1;
            int index2;
            int index3;

            trimesh_face()
            {
                index1 = -1;
                index2 = -1;
                index3 = -1;
            }

            trimesh_face(int ind1, int ind2, int ind3)
            {
                index1 = ind1;
                index2 = ind2;
                index3 = ind3;
            }

            void get(int* f)
            {
                f[0] = index1;
                f[1] = index2;
                f[2] = index3;
            }

        }face_t;

        /**
         * This class is responsible for handling all trimesh data
         * and trimesh data interactions for different visualization and physics tools.
         *
         * @brief <b> Trimesh representation class </b>
         * @authors Athanasios Krontiris, Kostas Bekris
         */
        class trimesh_t
        {
        private:
            /** @brief Vertex location information */
            std::vector<vector_t> vertices;

            /** @brief Vertex information of the faces */
            std::vector<face_t>    faces;

        public:

            double min_x,max_x,min_y,max_y,min_z,max_z;

            trimesh_t();
            ~trimesh_t();
            trimesh_t(const trimesh_t& tri);

            /**
             * Copies trimesh information in the parameter.
             *
             * @brief Copies trimesh information in the parameter.
             * @param tri The trimesh to be copied.
             */
            void copy( const trimesh_t* tri );

            /**
             * Clears the vectors \c vertices and \c faces.
             *
             * @brief Clears the vectors \c vertices and \c faces.
             */
            void clear();

            /**
             * Gives const access to the \c vertices class member.
             *
             * @brief Gives const access to the \c vertices class member.
             * @return A const reference of the \c vertices class member.
             */
            const std::vector<vector_t>& get_vertices() const;

            /**
             * Gives  access to the \c vertices class member.
             *
             * @brief Gives access to the \c vertices class member.
             * @return A reference of the \c vertices class member.
             */
            std::vector<vector_t>& get_vertices();

            /**
             * Gets the size of the \c vertices class member.
             *
             * @brief Gets the size of the \c vertices class member.
             * @return Returns the size of \c vertices.
             */
            int get_vertices_size() const;

            /**
             * Gives const access to the \c faces class member.
             *
             * @brief Gives const access to the \c faces class member.
             * @return A const reference of the \c faces class member.
             */
            const std::vector<face_t>& get_faces() const;

            /**
             * Gives access to the \c faces class member.
             *
             * @brief Gives access to the \c faces class member.
             * @return A reference of the \c faces class member.
             */
            std::vector<face_t>& get_faces();

            /**
             * Gets a specific face based on an index.
             *
             * @brief Gets a specific face based on an index.
             * @param index The index of the face to get.
             * @param f A pointer to the face.
             */
            void get_face_at(int index, face_t* f) const;

            /**
             * Returns the size of the \c faces class member.
             *
             * @brief Returns the size of the \c faces class member.
             * @return the size of the \c faces class member.
             */
            int get_faces_size() const;

            /**
             * Returns the vertices of each face along with their corresponding index.
             *
             * @brief Returns the vertices of each face along with their corresponding index.
             * @param vert The vertex information of the faces.
             * @param indices The indices of the faces.
             * @return The number of vertices.
             */
            int get_vertices_and_indeces(double** vert, int* indices);

            /**
             * Gets a specific vertex located at an index.
             *
             * @brief Gets a specific vertex located at an index.
             * @param place The index of the vertex to get.
             * @param vertex The vertex to return.
             */
            void get_vertex_at(size_t place, vector_t* vertex) const;

            /**
             * Gets a specific vertex located at an index.
             *
             * @brief Gets a specific vertex located at an index.
             * @param place The index of the vertex to get.
             * @return  The vertex to return.
             */
            const vector_t* get_vertex_at(size_t place) const;

            /**
             * Move the center of mass related to the scaling values.
             *
             * @brief Move the center of mass related to the scaling values.
             * @param scale Determines the axis to scale the trimesh with.
             */
            void move_to_zero_and_expand(const vector_t* scale);

            /**
             * Move the center of mass at the new point..
             *
             * @brief Move the center of mass..
             *
             * @param scale Determines the new location of the center of mass.
             */
            void place_center_of_mass(const vector_t* place);

            /**
             * Creates a sphere trimesh with the specified radius.
             *
             * @brief Creates a sphere trimesh with the specified radius.
             * @param radius The radius of the sphere to be created.
             */
            void create_sphere_trimesh( double radius );

            /**
             * Creates a box trimesh with the specified dimensions.
             *
             * @brief Creates a box trimesh with the specified dimensions.
             * @param lx The x-axis length.
             * @param ly The y-axis length.
             * @param lz The z-axis length.
             */
            void create_box_trimesh( double lx, double ly, double lz );

            /**
             * Creates a cone trimesh with the specified radius and height.
             *
             * @brief Creates a cone trimesh with the specified radius and height.
             * @param radius The cone's radius.
             * @param height The cone's height.
             */
            void create_cone_trimesh( double radius, double height );

            /**
             * Creates a cylinder trimesh with the specified radius and height.
             *
             * @brief Creates a cylinder trimesh with the specified radius and height.
             * @param radius The radius of the cylinder.
             * @param height The height of the cylinder.
             */
            void create_cylinder_trimesh( double radius, double height );

            /**
             * Creates an open cylinder trimesh with the specified radius and height.
             *
             * @brief Creates an open cylinder trimesh with the specified radius and height.
             * @param radius The radius of the open cylinder.
             * @param height The height of the open cylinder.
             */
            void create_open_cylinder_trimesh( double radius, double height );

            /**
             * Creates a capsule trimesh with the specified radius and height.
             *
             * @brief Creates a capsule trimesh with the specified radius and height.
             * @param radius The radius of the capsule.
             * @param height The height of the capsule.
             */
            void create_capsule_trimesh( double radius, double height );

            /**
             * Creates a triangle trimesh from the three vector_t's.
             *
             * @brief Creates a triangle trimesh from the three vertices.
             * @param v1 One of the triangle's sides.
             * @param v2 One of the triangle's sides.
             * @param v3 One of the triangle's sides.
             */
            void create_triangle_trimesh( const vector_t* v1, const vector_t* v2, const vector_t* v3 );

            /**
             * Creates a quadrilateral trimesh from the four vector_t's.
             *
             * @brief Creates a quadrilateral trimesh from the four vector_t's.
             * @param v1 One of the quad's sides.
             * @param v2 One of the quad's sides.
             * @param v3 One of the quad's sides.
             * @param v4 One of the quad's sides.
             */
            void create_quad_trimesh( const vector_t* v1, const vector_t* v2, const vector_t* v3, const vector_t* v4);

            /**
             * Loads a mesh from a file.
             *
             * @brief Loads a mesh from a file.
             * @param filename The name of the mesh file.
             */
            void create_mesh_from_file(const std::string& filename);

            /**
             * Loads a heightmap from a file.
             *
             * @brief Loads a heightmap from a file.
             * @param filename The name of the heightmap file.
             */
            void create_heightmap_from_file(const std::string& filename);

            /**
             * Directly constructs the trimesh from a list of triangle points.
             *
             * @brief Directly construct the trimesh from point data.
             */
            void create_from_vector( const std::vector< double >& triangles );

            friend std::ostream& operator<<( std::ostream& out, const trimesh_t& mesh );
        };
    }
}
#endif
