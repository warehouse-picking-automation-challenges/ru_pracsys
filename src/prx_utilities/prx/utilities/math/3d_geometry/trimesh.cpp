/**
 * @file trimesh.cpp
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

#include "prx/utilities/math/3d_geometry/trimesh.hpp"
#include <fstream>

#ifndef OLD_ASSIMP
#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#else
#include "assimp/assimp.hpp"
#include "assimp/aiPostProcess.h"
#include "assimp/aiScene.h"
#endif

namespace prx
{
    namespace util
    {

        trimesh_t::trimesh_t()
        {
            min_x = min_y = min_z = 999999;
            max_x = max_y = max_z = -999999;
        }

        trimesh_t::~trimesh_t()
        {
            //PRX_LOG_DEBUG("-- Destroy the trimesh_t");
        }

        trimesh_t::trimesh_t(const trimesh_t& tri)
        {
            vertices = tri.vertices;
            faces = tri.faces;
        }

        void trimesh_t::copy(const trimesh_t* tri)
        {
            //Clear out the current trimesh
            clear();
            //Copy the input trimesh's parameters
            vertices = tri->vertices;
            faces = tri->faces;
        }

        void trimesh_t::clear()
        {
            vertices.clear();
            faces.clear();
        }

        const std::vector<vector_t>& trimesh_t::get_vertices() const
        {
            return vertices;
        }

        int trimesh_t::get_vertices_size() const
        {
            return (int) vertices.size();
        }

        std::vector<vector_t>& trimesh_t::get_vertices()
        {
            return vertices;
        }

        const std::vector<face_t>& trimesh_t::get_faces() const
        {
            return faces;
        }

        std::vector<face_t>& trimesh_t::get_faces()
        {
            return faces;
        }

        void trimesh_t::get_face_at(int index, face_t* f) const
        {
            f->index1 = faces[index].index1;
            f->index2 = faces[index].index2;
            f->index3 = faces[index].index3;
        }

        int trimesh_t::get_faces_size() const
        {
            return (int) faces.size();
        }

        int trimesh_t::get_vertices_and_indeces(double** vert, int* indices)
        {
            int sz = vertices.size();
            int sz_face = faces.size();
            int i;

            vert = new double*[sz];
            for (i = 0; i < sz; ++i)
                vert[i] = new double[3];

            indices = new int(sz_face * 3);

            for (i = 0; i < sz; ++i)
                vertices[i].get(vert[i]);

            for (i = 0; i < sz_face; ++i) {
                indices[i * 3] = faces[i].index1;
                indices[i * 3 + 1] = faces[i].index2;
                indices[i * 3 + 2] = faces[i].index3;
            }
            return (sz_face * 3);
        }

        void trimesh_t::get_vertex_at(size_t place, vector_t* vertex) const
        {
            PRX_ASSERT(place < vertices.size());
            *vertex = vertices[place];
        }

        const vector_t* trimesh_t::get_vertex_at(size_t place) const
        {
            PRX_ASSERT(place < vertices.size());
            return &vertices[place];
        }

        void trimesh_t::move_to_zero_and_expand(const vector_t* scale)
        {
            vector_t center_mass(3);

            for (std::vector<vector_t>::iterator iter = vertices.begin(); iter != vertices.end(); ++iter)
                center_mass += *iter;
            center_mass /= vertices.size();

            for (std::vector<vector_t>::iterator iter = vertices.begin(); iter != vertices.end(); ++iter)
            {
                //(*iter).print();
                *iter *= *scale;
            }
        }

        void trimesh_t::place_center_of_mass(const vector_t* place)
        {
            vector_t center_mass(3);

            for (std::vector<vector_t>::iterator iter = vertices.begin(); iter != vertices.end(); ++iter)
                center_mass += *iter;
            center_mass /= vertices.size();

            for (std::vector<vector_t>::iterator iter = vertices.begin(); iter != vertices.end(); ++iter) {
                *iter -= center_mass;
                *iter += *place;
            }
        }

        void trimesh_t::create_sphere_trimesh(double radius)
        {
            vector_t v(3);
            double angle, height_angle;
            for (int i = 0; i < 31; i++)
            {
                height_angle = (((double) (15 - i)) * PRX_PI / 30.0);
                for (int j = 0; j < 30; j++)
                {
                    angle = ((double) (j * 2.0 * PRX_PI)) / 30.0;

                    v[0] = cos(angle) * cos(height_angle) * radius;
                    v[1] = sin(height_angle) * radius;
                    v[2] = sin(angle) * cos(height_angle) * radius;
                    vertices.push_back(v);
                }
            }

            for (int i = 0; i < 31 - 1; i++)
            {
                for (int j = 0; j < 30 - 1; j++)
                {
                    faces.push_back(face_t(i * 30 + j, (i + 1)*30 + j, (i + 1)*30 + j + 1));
                    faces.push_back(face_t(i * 30 + j, (i + 1)*30 + (j + 1), i * 30 + (j + 1)));
                }

                faces.push_back(face_t(i * 30 + 29, (i + 1)*30 + 29, (i + 1)*30));
                faces.push_back(face_t(i * 30 + 29, (i + 1)*30, i * 30));
            }
        }

        void trimesh_t::create_box_trimesh(double lx, double ly, double lz)
        {
            vector_t v(3);
            for (int i = 0; i < 8; i++)
            {
                v[0] = (lx / 2.0);
                v[1] = (ly / 2.0);
                v[2] = (lz / 2.0);

                if (i % 2 == 1)
                    v[2] = -v[2];
                if (i % 4 >= 2)
                    v[1] = -v[1];
                if (i >= 4)
                    v[0] = -v[0];

                vertices.push_back(v);
            }

            //    PRX_LOG_DEBUG("dims : %lf %lf %lf", lx,ly,lz);
            //    foreach(vector_t vec, vertices)
            //        vec.print();

            faces.push_back(face_t(0, 1, 2));
            faces.push_back(face_t(1, 3, 2));
            faces.push_back(face_t(4, 0, 6));
            faces.push_back(face_t(0, 2, 6));
            faces.push_back(face_t(5, 4, 7));
            faces.push_back(face_t(4, 6, 7));
            faces.push_back(face_t(1, 5, 3));
            faces.push_back(face_t(5, 7, 3));
            faces.push_back(face_t(5, 1, 4));
            faces.push_back(face_t(1, 0, 4));
            faces.push_back(face_t(7, 6, 2));
            faces.push_back(face_t(3, 7, 2));
        }

        void trimesh_t::create_cone_trimesh(double radius, double height)
        {
            double points = (int) (2 * PRX_PI * radius / 5);
            double angle;
            vector_t v(3);

            vertices.push_back(vector_t(0.0, 0.0, height));
            vertices.push_back(vector_t(0.0, 0.0, 0.0));
            vertices.push_back(vector_t(radius, 0.0, 0.0));

            for (int i = 3; i <= points + 3; ++i)
            {
                angle = ((double) ((i-2) * 2.0 * PRX_PI)) / points;
                v.set(cos(angle) * radius, sin(angle) * radius, 0.0);
                vertices.push_back(v);
                faces.push_back(face_t(0, i - 1, i));
                faces.push_back(face_t(1, i, i - 1));
            }
            faces.push_back(face_t(0, points, 2));
            faces.push_back(face_t(1, 2, points));
            v.set(0, 0, 0);
            place_center_of_mass(&v);
        }

        void trimesh_t::create_cylinder_trimesh(double radius, double height)
        {
            double angle;
            vector_t v1(3), v2(3);
            for (int i = 0; i < 62; ++i)
                vertices.push_back(vector_t(3));

            for (int i = 0; i < 30; i++)
            {
                angle = ((double) (i * 2.0 * PRX_PI)) / 30.0;


                v1[0] = v2[0] = cos(angle) * radius;
                v1[1] = v2[1] = sin(angle) * radius;
                v1[2] = height / 2.0;
                v2[2] = -height / 2.0;

                vertices[i] = v1;
                vertices[31 + i] = v2;
            }

            v1[0] = 0;
            v1[1] = 0;
            v1[2] = height / 2.0;
            vertices[30] = v1;

            v1[0] = 0;
            v1[1] = 0;
            v1[2] = -height / 2.0;
            vertices[60] = v1;

            for (int i = 0; i < 29; i++)
            {
                faces.push_back(face_t(i, i + 31, i + 1));
                faces.push_back(face_t(i + 1, i + 31, i + 1 + 31));
            }
            faces.push_back(face_t(29, 60, 0));
            faces.push_back(face_t(0, 60, 31));


            for (int i = 0; i < 29; i++)
                faces.push_back(face_t(i, 30, i + 1));
            faces.push_back(face_t(29, 30, 0));

            for (int i = 0; i < 29; i++)
                faces.push_back(face_t(i + 31, 61, i + 1 + 31));
            faces.push_back(face_t(60, 61, 31));
        }

        void trimesh_t::create_open_cylinder_trimesh(double radius, double height)
        {
            vector_t v1(3), v2(3);
            double angle;

            for (int i = 0; i < 60; ++i)
                vertices.push_back(vector_t(3));

            for (int i = 0; i < 30; i++)
            {
                angle = ((double) (i * 2.0 * PRX_PI)) / 30.0;
                v1[0] = v2[0] = cos(angle) * radius;
                v1[1] = v2[1] = sin(angle) * radius;
                v1[2] = height / 2.0;
                v2[2] = -height / 2.0;

                vertices[i] = v1;
                vertices[30 + i] = v2;
            }

            for (int i = 0; i < 29; i++)
            {
                faces.push_back(face_t(i, i + 30, i + 1));
                faces.push_back(face_t(i + 1, i + 30, i + 1 + 30));
            }
            faces.push_back(face_t(29, 59, 0));
            faces.push_back(face_t(0, 59, 30));
        }

        void trimesh_t::create_capsule_trimesh(double radius, double height)
        {
            double angle, height_angle;
            int i, j;
            vector_t v(3);

            for (i = 0; i < 16; i++)
            {
                height_angle = (((double) (15 - i)) * PRX_PI / 30.0);
                for (j = 0; j < 30; j++)
                {
                    angle = ((double) (j * 2.0 * PRX_PI)) / 30.0;

                    v[0] = cos(angle) * cos(height_angle) * radius;
                    v[1] = sin(angle) * cos(height_angle) * radius;
                    v[2] = sin(height_angle) * radius + height / 2.0;
                    vertices.push_back(v);
                }
            }

            for (i = 16; i < 32; i++)
            {
                height_angle = (((double) (16 - i)) * PRX_PI / 30.0);
                for (j = 0; j < 30; j++)
                {
                    angle = ((double) (j * 2.0 * PRX_PI)) / 30.0;

                    v[0] = cos(angle) * cos(height_angle) * radius;
                    v[1] = sin(angle) * cos(height_angle) * radius;
                    v[2] = sin(height_angle) * radius - height / 2.0;
                    vertices.push_back(v);
                }
            }

            for (int i = 0; i < 32 - 1; i++)
            {
                for (int j = 0; j < 30 - 1; j++)
                {
                    faces.push_back(face_t(i * 30 + j, (i + 1)*30 + j, (i + 1)*30 + j + 1));
                    faces.push_back(face_t(i * 30 + j, (i + 1)*30 + (j + 1), i * 30 + (j + 1)));

                }

                faces.push_back(face_t(i * 30 + 29, (i + 1)*30 + 29, (i + 1)*30));
                faces.push_back(face_t(i * 30 + 29, (i + 1)*30, i * 30));
            }
        }

        void trimesh_t::create_triangle_trimesh(const vector_t* v1, const vector_t* v2, const vector_t* v3)
        {
            vertices.push_back(*v1);
            vertices.push_back(*v2);
            vertices.push_back(*v3);

            faces.push_back(face_t(0, 1, 2));
        }

        void trimesh_t::create_quad_trimesh(const vector_t* v1, const vector_t* v2, const vector_t* v3, const vector_t* v4)
        {
            vertices.push_back(*v1);
            vertices.push_back(*v2);
            vertices.push_back(*v3);
            vertices.push_back(*v4);

            faces.push_back(face_t(0, 2, 1));
            faces.push_back(face_t(0, 3, 2));
        }

        void trimesh_t::create_mesh_from_file(const std::string& filename)
        {
            Assimp::Importer importer;
            vertices.clear();
            faces.clear();
            char* name=getenv("PRACSYS_MODELS_PATH");
            std::string fn=(std::string)name+"/"+filename;

            PRX_DEBUG_COLOR("IMPORTING MODEL FILE " << fn.c_str(), PRX_TEXT_GREEN );

            const aiScene* scene = importer.ReadFile(fn.c_str(),
                                                     aiProcess_Triangulate            |
                                                     aiProcess_JoinIdenticalVertices  |
                                                     aiProcess_SortByPType            |
                                                     aiProcess_OptimizeGraph          |
                                                     aiProcess_OptimizeMeshes         |
                                                     aiProcess_SortByPType);

            if(scene == NULL)
            {
                PRX_FATAL_S("Can't load mesh.  [" << importer.GetErrorString() << "]" );
            }

            const aiNode *node=scene->mRootNode;

            aiMatrix4x4 transform;
            aiVector3D aiV;
            vector_t v(3);
            transform *= node->mTransformation;
            int val=0;
            for (unsigned int i = 0 ; i < node->mNumMeshes; ++i)
            {
                val=vertices.size();
                const aiMesh* a = scene->mMeshes[node->mMeshes[i]];
                for (unsigned int i = 0 ; i < a->mNumVertices ; ++i)
                {
                    aiV= transform * a->mVertices[i]; //
                    v[0]=aiV[0];
                    v[1]=aiV[1];
                    v[2]=aiV[2];


                    if(v[0]<min_x)
                        min_x = v[0];
                    if(v[0]>max_x)
                        max_x = v[0];
                    //y
                    if(v[1]<min_y)
                        min_y = v[1];
                    if(v[1]>max_y)
                        max_y = v[1];
                    //z
                    if(v[2]<min_z)
                        min_z = v[2];
                    if(v[2]>max_z)
                        max_z = v[2];

                    vertices.push_back(v);
                }
                for (unsigned int i = 0 ; i < a->mNumFaces ; ++i)
                {
                    v[0]=a->mFaces[i].mIndices[0]+val;
                    v[1]=a->mFaces[i].mIndices[1]+val;
                    v[2]=a->mFaces[i].mIndices[2]+val;
                    faces.push_back(face_t(v[0],v[1],v[2]));
                }
            }

            PRX_DEBUG_COLOR("NUMBER OF VERTICES " << (int)vertices.size() << " NUMBER OF FACES " << (int)faces.size(), PRX_TEXT_GREEN );

        }
        void trimesh_t::create_heightmap_from_file(const std::string& filename)
        {
            Assimp::Importer importer;
            vertices.clear();
            faces.clear();
            char* name=getenv("PRACSYS_MODELS_PATH");
            std::string fn=(std::string)name+"/"+filename;

            std::ifstream fin;
            fin.open(fn.c_str());

            vector_t point(3);
            int number_of_points;
            fin>>number_of_points;
            double x,y,z;
            for(int i=0;i<number_of_points;i++)
            {
                fin>>x;
                fin>>y;
                fin>>z;
                point.set(x,y,z);
                vertices.push_back(point);
            }
            int side_size = sqrt(number_of_points);
            vector_t normal,vertex1,vertex2,vertex3;
            normal.resize(3);
            vertex1 = vertices[0];
            vertex2 = vertices[1];
            vertex3 = vertices[side_size+1];
            normal.cross_product(vertex2-vertex1,vertex3-vertex1);
            bool normal_up = (normal[2] > 0);
            if(!normal_up)
            {
                for(int i=0;i<side_size-1;i++)
                {
                    for(int j=0;j<side_size-1;j++)
                    {
                        faces.push_back(face_t(j+i*side_size, j+(i+1)*side_size, j+(i+1)*side_size+1));
                        faces.push_back(face_t(j+i*side_size, j+(i+1)*side_size+1, j+i*side_size+1));
                    }
                }
            }
            else
            {
                for(int i=0;i<side_size-1;i++)
                {
                    for(int j=0;j<side_size-1;j++)
                    {
                        faces.push_back(face_t(j+i*side_size, j+i*side_size+1, j+(i+1)*side_size+1));
                        faces.push_back(face_t(j+i*side_size, j+(i+1)*side_size+1, j+(i+1)*side_size));
                    }
                }

            }

            fin.close();
            // vertices.push_back(*v1);
            // vertices.push_back(*v2);
            // vertices.push_back(*v3);

            // faces.push_back(face_t(0, 1, 2));

            PRX_DEBUG_COLOR("NUMBER OF VERTICES " << (int)vertices.size() << " NUMBER OF FACES " << (int)faces.size(), PRX_TEXT_GREEN );

        }

        void trimesh_t::create_from_vector( const std::vector<double>& triangles )
        {
            PRX_ASSERT(triangles.size() % 9 == 0);

            unsigned nr_verts = 0;

            vector_t v1(0, 0, 0);
            vector_t v2(0, 0, 0);
            vector_t v3(0, 0, 0);

            //For each face
            for( unsigned i=0; i<triangles.size(); i+=9 )
            {
                v1.set( triangles[i], triangles[i+1], triangles[i+2] );
                v2.set( triangles[i+3], triangles[i+4], triangles[i+5] );
                v3.set( triangles[i+6], triangles[i+7], triangles[i+8] );
                vertices.push_back( v1 );
                vertices.push_back( v2 );
                vertices.push_back( v3 );

                faces.push_back(face_t(nr_verts, nr_verts+1, nr_verts+2));
                nr_verts += 3;
            }
        }

        std::ostream& operator<<( std::ostream& out, const face_t& face )
        {
            out << face.index1 << " " << face.index2 << " " << face.index3;
            return out;
        }

        std::ostream& operator<<( std::ostream& out, const trimesh_t& mesh )
        {
            for (size_t i = 0; i < mesh.vertices.size(); ++i)
                out << mesh.vertices[i] << "\n";

            for (size_t i = 0; i < mesh.faces.size(); ++i)
                out << mesh.faces[i];

            return out;
        }
    }
}
