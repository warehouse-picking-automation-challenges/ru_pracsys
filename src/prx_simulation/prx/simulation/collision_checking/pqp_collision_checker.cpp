/**
 * @file pqp_collision_checker.cpp
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

#include "prx/simulation/collision_checking/pqp_collision_checker.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/simulation/system_ptr.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::pqp_collision_checker_t, prx::sim::collision_checker_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        pqp_collision_checker_t::pqp_collision_checker_t()
        {
            collision_list = NULL;
        }

        pqp_collision_checker_t::~pqp_collision_checker_t()
        {
            for( models_map_iter = models_map.begin(); models_map_iter != models_map.end(); ++models_map_iter )
                delete models_map_iter->second;

            models_map.clear();

        }

        void pqp_collision_checker_t::link_collision_list(collision_list_t* list)
        {
            //This is assuming that any changes to the environment (obstacles and added/removed systems) are reflected in
            // new collision lists that are linked.

            collision_checker_t::link_collision_list(list);
            body_cache.clear();

            foreach(collision_pair_t pair, collision_list->get_body_pairs())
            {
                body_cache.push_back(std::pair<pqp_info_t*, pqp_info_t*>(models_map[pair.first], models_map[pair.second]));
            }

        }

        void pqp_collision_checker_t::set_configuration(const std::string& name, const config_t& config)
        {
            models_map[name]->update_matrices(config);
        }

        void pqp_collision_checker_t::add_body(const std::string& name, const geometry_t& geometry, system_t* plant, bool is_obstacle)
        {
            if( models_map[name] == NULL )
            {
                models_map[name] = new pqp_info_t(plant);
                models_map[name]->name = name;
                create_model(name, geometry);
            }
        }

        void pqp_collision_checker_t::remove_body(const std::string& name)
        {
            if( models_map[name] != NULL )
            {
                models_map.erase(name);
                models_map[name] = NULL;
            }
        }

        bool pqp_collision_checker_t::in_collision()
        {
            if( collision_list != NULL )
            {
                unsigned size = body_cache.size();
                for( unsigned i = 0; i < size; i++ )
                    if( in_collision(body_cache[i].first, body_cache[i].second) )
                        return true;
            }

            return false;
        }

        //TODO: Can this in_collision() function be used at the general level?
        bool pqp_collision_checker_t::in_collision( collision_list_t* list )
        {
            if( list != NULL )
            {
                foreach(collision_pair_t pair, list->get_body_pairs())
                {
                    if( in_collision(pair.first, pair.second) )
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        bool pqp_collision_checker_t::near_collision(double eps)
        {
            if( collision_list != NULL )
            {
                unsigned size = body_cache.size();
                for( unsigned i = 0; i < size; i++ )
                    if( near_collision(eps, body_cache[i].first, body_cache[i].second) )
                        return true;
            }
            return false;
        }

        double pqp_collision_checker_t::get_clearance()
        {
            double clearance_found = PRX_INFINITY;
            if( collision_list != NULL )
            {
                unsigned size = body_cache.size();
                for( unsigned i = 0; i < size; i++ )
                {
                    double found = get_clearance(clearance_found, body_cache[i].first, body_cache[i].second);
                    clearance_found = PRX_MINIMUM(found, clearance_found);
                }
            }
            return clearance_found;
        }

        collision_list_t* pqp_collision_checker_t::colliding_bodies()
        {
            // PRX_DEBUG_COLOR("::: pqp_collision_checker_t::colliding_bodies()", PRX_TEXT_LIGHTGRAY);
            colliding_bodies_list->clear();
            collision_pair_t pair;
            //    PRX_DEBUG_S("Size : " << static_cast<vector_collision_list_t*>(collision_list)->size());
            if( collision_list != NULL )
            {
                for( unsigned i = 0; i < body_cache.size(); i++ )
                    if( in_collision(body_cache[i].first, body_cache[i].second) )
                    {
                        //                        PRX_INFO_S(body_cache[i].first->name << " :" << body_cache[i].first->translation_matrix[0] << " , " << body_cache[i].first->translation_matrix[1] << " , " << body_cache[i].first->translation_matrix[2]);
                        //                        PRX_INFO_S(body_cache[i].second->name << " :" << body_cache[i].second->translation_matrix[0] << " , " << body_cache[i].second->translation_matrix[1] << " , " << body_cache[i].second->translation_matrix[2]);
                        //
                        //                        PRX_INFO_S(body_cache[i].first->name << " :" << body_cache[i].first->rotation_matrix[0][0] << " , " << body_cache[i].first->rotation_matrix[0][1] << " , " << body_cache[i].first->rotation_matrix[0][2]);
                        //                        PRX_INFO_S(body_cache[i].first->name << " :" << body_cache[i].first->rotation_matrix[1][0] << " , " << body_cache[i].first->rotation_matrix[1][1] << " , " << body_cache[i].first->rotation_matrix[1][2]);
                        //                        PRX_INFO_S(body_cache[i].first->name << " :" << body_cache[i].first->rotation_matrix[2][0] << " , " << body_cache[i].first->rotation_matrix[2][1] << " , " << body_cache[i].first->rotation_matrix[2][2]);

                        pair.first = body_cache[i].first->name;
                        pair.second = body_cache[i].second->name;
                        colliding_bodies_list->add_new_pair(pair);
                    }
                //        foreahc(collision_pair_t pair, collision_list->get_body_pairs())
                //        {
                ////            PRX_DEBUG_S("Pair to check : " << pair.first << " -> " << pair.second);
                //            if(in_collision(pair.first,pair.second))
                //                colliding_bodies_list->add_new_pair(pair);
                //        }
            }
            // PRX_DEBUG_COLOR("::: <done", PRX_TEXT_CYAN);
            return colliding_bodies_list;
        }

        collision_list_t* pqp_collision_checker_t::colliding_bodies( collision_list_t* list )
        {
            colliding_bodies_list->clear();

            if( list != NULL )
            {
                foreach(collision_pair_t pair, list->get_body_pairs())
                {
                    if( in_collision(pair.first, pair.second) )
                    {
                        colliding_bodies_list->add_new_pair( pair );
                    }
                }
            }

            return colliding_bodies_list;
        }

        collision_list_t* pqp_collision_checker_t::near_colliding_bodies(double eps)
        {

            colliding_bodies_list->clear();
            collision_pair_t pair;
            if( collision_list != NULL )
            {
                for( unsigned i = 0; i < body_cache.size(); i++ )
                    if( near_collision(eps, body_cache[i].first, body_cache[i].second) )
                    {
                        pair.first = body_cache[i].first->name;
                        pair.second = body_cache[i].second->name;
                        colliding_bodies_list->add_new_pair(pair);
                    }
            }
            return colliding_bodies_list;
        }

        void pqp_collision_checker_t::create_model(const std::string& name, const geometry_t& geometry)
        {
            vector_t vertex; // tmp variable
            face_t face; // tmp variable

            trimesh_t trimesh;
            geometry.get_trimesh(&trimesh);
            PRX_DEBUG_COLOR("create the model for : " << name, PRX_TEXT_GREEN);
            //    geometry.print();
            PQP_Model* model = models_map[name]->model;

            model->BeginModel();

            double radius = 0;
            for( int i = 0; i < trimesh.get_faces_size(); ++i )
            {
                trimesh.get_face_at(i, &face);
                PQP_REAL first[3], second[3], third[3];

                trimesh.get_vertex_at(face.index1, &vertex);
                radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                first[0] = vertex[0];
                first[1] = vertex[1];
                first[2] = vertex[2];

                trimesh.get_vertex_at(face.index2, &vertex);
                radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                second[0] = vertex[0];
                second[1] = vertex[1];
                second[2] = vertex[2];

                trimesh.get_vertex_at(face.index3, &vertex);
                radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                third[0] = vertex[0];
                third[1] = vertex[1];
                third[2] = vertex[2];

                model->AddTri(first, second, third, i);
            }
            models_map[name]->broadphase_radius = radius;

            model->EndModel();
        }

        bool pqp_collision_checker_t::in_collision(pqp_info_t* info1, pqp_info_t* info2)
        {
#ifndef NDEBUG
            if( info1 == NULL )
                throw invalid_system_pair("A system does not exist in the collision_checker");
            if( info2 == NULL )
                throw invalid_system_pair("A system does not exist in the collision_checker");

#endif
            if( info1->system->is_active() && info2->system->is_active() )
            {
                double dx = info1->translation_matrix[0] - info2->translation_matrix[0];
                double dy = info1->translation_matrix[1] - info2->translation_matrix[1];
                double dz = info1->translation_matrix[2] - info2->translation_matrix[2];
                double dr = info1->broadphase_radius + info2->broadphase_radius;

                if( dx * dx + dy * dy + dz * dz < dr * dr )
                {
                    //                PRX_ERROR_S("pair:" << info1->system->get_pathname() << "->" << info2->system->get_pathname());
                    PQP_CollideResult collision_result;
                    PQP_Collide(&collision_result,
                                info1->rotation_matrix, info1->translation_matrix, info1->model,
                                info2->rotation_matrix, info2->translation_matrix, info2->model,
                                PQP_FIRST_CONTACT);

                    return collision_result.Colliding();
                }
            }
            return false;
        }

        bool pqp_collision_checker_t::in_collision(const std::string& name1, const std::string& name2)
        {
            pqp_info_t* info1 = models_map[name1];
            pqp_info_t* info2 = models_map[name2];
            return in_collision(info1, info2);
            // #ifndef NDEBUG
            //             if( info1 == NULL )
            //                 throw invalid_system_pair("A system does not exist in the collision_checker");
            //             if( info2 == NULL )
            //                 throw invalid_system_pair("A system does not exist in the collision_checker");
            // #endif
            //             if( info1->system->is_active() && info2->system->is_active()
            //                 &&  (( (info1->translation_matrix[0] -info2->translation_matrix[0])*(info1->translation_matrix[0] -info2->translation_matrix[0]) +
            //                       (info1->translation_matrix[1] -info2->translation_matrix[1])*(info1->translation_matrix[1] -info2->translation_matrix[1]) +
            //                       (info1->translation_matrix[2] -info2->translation_matrix[2])*(info1->translation_matrix[2] -info2->translation_matrix[2])
            //                     ) < (info1->broadphase_radius+info2->broadphase_radius)*(info1->broadphase_radius+info2->broadphase_radius))
            //               )
            //             {
            // //                PRX_ERROR_S("pair:" << name1 << "->" << name2);
            //                 PQP_CollideResult collision_result;
            //                 PQP_Collide(&collision_result,
            //                             info1->rotation_matrix, info1->translation_matrix, info1->model,
            //                             info2->rotation_matrix, info2->translation_matrix, info2->model,
            //                             PQP_FIRST_CONTACT);

            //                 return collision_result.Colliding();
            //             }
            //             return false;
        }

        bool pqp_collision_checker_t::near_collision(double eps, pqp_info_t* info1, pqp_info_t* info2)
        {
#ifndef NDEBUG
            if( info1 == NULL )
                throw invalid_system_pair("A system does not exist in the collision_checker");
            if( info2 == NULL )
                throw invalid_system_pair("A system does not exist in the collision_checker");
#endif
            if( info1->system->is_active() && info2->system->is_active() )
            {
                double dx = info1->translation_matrix[0] - info2->translation_matrix[0];
                double dy = info1->translation_matrix[1] - info2->translation_matrix[1];
                double dz = info1->translation_matrix[2] - info2->translation_matrix[2];
                double drad = info1->broadphase_radius + info2->broadphase_radius;

                if( dx * dx + dy * dy + dz * dz < drad * drad + eps * eps )
                {
                    PQP_DistanceResult dr;
                    PQP_Distance(&dr, info1->rotation_matrix, info1->translation_matrix, info1->model,
                                 info2->rotation_matrix, info2->translation_matrix, info2->model,
                                 0.01, PRX_ZERO_CHECK); // Probably need to figure out exactly what these correspond to.

                    return dr.Distance() < eps;
                }
            }
            return false;
        }

        double pqp_collision_checker_t::get_clearance(double min_so_far, pqp_info_t* info1, pqp_info_t* info2)
        {
#ifndef NDEBUG
            if( info1 == NULL )
                throw invalid_system_pair("A system does not exist in the collision_checker");
            if( info2 == NULL )
                throw invalid_system_pair("A system does not exist in the collision_checker");
#endif
            if( info1->system->is_active() && info2->system->is_active() )
            {
                double dx = info1->translation_matrix[0] - info2->translation_matrix[0];
                double dy = info1->translation_matrix[1] - info2->translation_matrix[1];
                double dz = info1->translation_matrix[2] - info2->translation_matrix[2];
                double drad = info1->broadphase_radius + info2->broadphase_radius;

                if( dx * dx + dy * dy + dz * dz < drad * drad + min_so_far * min_so_far )
                {
                    PQP_DistanceResult dr;
                    PQP_Distance(&dr, info1->rotation_matrix, info1->translation_matrix, info1->model,
                                 info2->rotation_matrix, info2->translation_matrix, info2->model,
                                 0.01, PRX_ZERO_CHECK); // Probably need to figure out exactly what these correspond to.

                    return dr.Distance();
                }
            }
            return PRX_INFINITY;
        }

        std::ostream& operator<<(std::ostream& output, const pqp_collision_checker_t& checker)
        {
            //    output << "Systems in the collision_checker" << std::endl;
            //    for(hash_t<std::string, pqp_info_t*>::const_iterator  iter= checker.models_map.begin(); iter != checker.models_map.end(); ++iter)
            //        output << iter->first << std::endl;
            //
            //    output << "\nWhite list for collisions: " << std::endl;


            if( checker.collision_list )
                checker.collision_list->print();

            output << std::endl;

            return output;
        }

    }
}
