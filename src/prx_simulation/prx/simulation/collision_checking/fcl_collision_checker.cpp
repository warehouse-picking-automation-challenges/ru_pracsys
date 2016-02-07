/**
 * @file fcl_collision_checker.cpp
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

#include "prx/simulation/collision_checking/fcl_collision_checker.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/simulation/system_ptr.hpp" 
#include "prx/utilities/definitions/sys_clock.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp> //adaptors

PLUGINLIB_EXPORT_CLASS(prx::sim::fcl_collision_checker_t, prx::sim::collision_checker_t)

namespace prx
{
    using namespace util;

    namespace sim
    {
        fcl_collision_checker_t::fcl_collision_checker_t()
        {
            collision_list = NULL;
            vis_array_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
            PRX_PRINT("FCL CC constructor succesfully created.", PRX_TEXT_CYAN);
        }

        fcl_collision_checker_t::~fcl_collision_checker_t()
        {
            for( models_map_iter = models_map.begin(); models_map_iter != models_map.end(); ++models_map_iter )
                delete models_map_iter->second;

            models_map.clear();

        }

        void fcl_collision_checker_t::set_configuration(const std::string& name, const config_t& config)
        {
            models_map[name]->update_matrices(config);
        }

        void fcl_collision_checker_t::add_body(const std::string& name, const geometry_t& geometry, system_t* plant, bool is_obstacle)
        {
            if( models_map[name] == NULL )
            {
                models_map[name] = new fcl_info_t(plant);
                create_model(name, geometry);
            }
        }

        void fcl_collision_checker_t::remove_body(const std::string& name)
        {
            if( models_map[name] != NULL )
            {
                models_map.erase(name);
                models_map[name] = NULL;
            }
        }

        bool fcl_collision_checker_t::in_collision()
        {
            if( collision_list != NULL )
            {
                unsigned size = body_cache.size();
                for( unsigned i = 0; i < size; i++ )
                    if( in_collision(body_cache[i].first, body_cache[i].second) )
                    {
                        return true;
                    }
            }

            return false;

            // // PRX_INFO_S("I am in the collision checker's in collision function");
            // if( collision_list != NULL )
            // {
            //     foreach(const collision_pair_t& pair, collision_list->get_body_pairs())
            //     {
            //         if( in_collision(pair.first, pair.second) )
            //         {
            //             return true;
            //         }
            //     }
            // }

            // return false;
        }

        bool fcl_collision_checker_t::in_collision( collision_list_t* list )
        {
            if( list != NULL )
            {
                foreach(const collision_pair_t& pair, list->get_body_pairs())
                {
                    if( in_collision(pair.first, pair.second) )
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        void fcl_collision_checker_t::link_collision_list(collision_list_t* list)
        {
            //This is assuming that any changes to the environment (obstacles and added/removed systems) are reflected in
            // new collision lists that are linked.

            collision_checker_t::link_collision_list(list);
            body_cache.clear();

            foreach(collision_pair_t pair, collision_list->get_body_pairs())
            {
                // if( models_map[pair.first] == NULL )
                // {
                //     PRX_FATAL_S("Unknown model map hash: " << pair.first);
                // }
                // if( models_map[pair.second] == NULL )
                // {
                //     PRX_FATAL_S("Unknown model map hash: " << pair.second);
                // }

                body_cache.push_back(std::pair<fcl_info_t*, fcl_info_t*>(models_map[pair.first], models_map[pair.second]));
            }

        }

        collision_list_t* fcl_collision_checker_t::colliding_bodies()
        {
            //DEBUGGING STUFFS
            // sys_clock_t clock;
            // clock.reset();

            colliding_bodies_list->clear();
            if( collision_list != NULL )
            {
                foreach(collision_pair_t pair, collision_list->get_body_pairs())
                {
                    if( in_collision(pair.first, pair.second) )
                        colliding_bodies_list->add_new_pair(pair);
                }
            }

            // PRX_PRINT("Colliding Bodies: [" << collision_list->size() << "]" << clock.measure(), PRX_TEXT_BROWN);

            return colliding_bodies_list;
        }

        collision_list_t* fcl_collision_checker_t::colliding_bodies( collision_list_t* list )
        {
            // sys_clock_t clock;
            // clock.reset();

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

            // PRX_PRINT("Colliding Bodies (list) [" << list->size() << "]: " << clock.measure(), PRX_TEXT_GREEN);

            return colliding_bodies_list;
        }

#define PRIMITIVES 1
        void fcl_collision_checker_t::create_model(const std::string& name, const geometry_t& geometry)
        {

// #ifdef FCL_FOUND
            geometry_type gt = geometry.get_type();
            if(PRIMITIVES && (gt <= 4 || gt ==6 ))
            {
                if(gt == PRX_SPHERE)
                {
                    double sphere;
                    geometry.get_sphere(sphere);
                    models_map[name]->model = new fcl::Sphere(sphere);
                }
                else if(gt == PRX_BOX)
                {
                    double x,y,z;
                    geometry.get_box(x,y,z);
                    models_map[name]->model = new fcl::Box(x,y,z);
                }
                else if(gt == PRX_CONE)
                {
                    double rad,z;
                    geometry.get_cone(rad,z);
                    models_map[name]->model = new fcl::Cone(rad,z);
                }
                else if(gt == PRX_CYLINDER)
                {
                    double rad,z;
                    geometry.get_cylinder(rad,z);
                    models_map[name]->model = new fcl::Cylinder(rad,z);                  
                }
                else if(gt == PRX_CAPSULE)
                {
                    double rad,z;
                    geometry.get_capsule(rad,z);
                    models_map[name]->model = new fcl::Capsule(rad,z);                
                }
                else
                {
                    PRX_FATAL_S("This should not happen! Trying to create FCL geometry ");
                }

                trimesh_t trimesh;
                geometry.get_trimesh(&trimesh);
                int nr_indices = trimesh.get_faces_size();
                vector_t vertex; // tmp variable
                face_t face; // tmp variable
                double radius=0;
                
                for( int i = 0; i < nr_indices; ++i )
                {
                    trimesh.get_face_at(i, &face);
                    trimesh.get_vertex_at(face.index1, &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                    trimesh.get_vertex_at(face.index2, &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                    trimesh.get_vertex_at(face.index3, &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                }
                
                models_map[name]->broadphase_radius = radius;
            }
            else if(gt==PRX_CLOUD)
            {

            }
            else
            {
                trimesh_t trimesh;
                geometry.get_trimesh(&trimesh);
                PRX_DEBUG_S("create the model for : " << name);
                //    geometry.print();
                models_map[name]->model = new FCL_Mesh();
                FCL_Mesh* model = static_cast<FCL_Mesh*>(models_map[name]->model);

                model->beginModel();

                int nr_indices = trimesh.get_faces_size();

                // if( nr_indices % 3 != 0 )
                //     PRX_FATAL_S("There is a problem with the model. \nIt is probably not made out of triangles\n Number of indices : " << nr_indices);

                std::vector<fcl::Triangle> triangles;
                std::vector <fcl::Vec3f> points;

                vector_t vertex; // tmp variable
                face_t face; // tmp variable
                double radius=0;
                
                for( int i = 0; i < nr_indices; ++i )
                {
                    trimesh.get_face_at(i, &face);
                    trimesh.get_vertex_at(face.index1, &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));

                    points.push_back(fcl::Vec3f(vertex[0], vertex[1], vertex[2]));

                    trimesh.get_vertex_at(face.index2, &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                    points.push_back(fcl::Vec3f(vertex[0], vertex[1], vertex[2]));

                    trimesh.get_vertex_at(face.index3, &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                    points.push_back(fcl::Vec3f(vertex[0], vertex[1], vertex[2]));

                    triangles.push_back(fcl::Triangle(points.size() - 3, points.size() - 2, points.size() - 1));
                }
                
                models_map[name]->broadphase_radius = radius;
                
                
                model->addSubModel(points, triangles);
                model->endModel();
                model->computeLocalAABB();
            }
// #endif
        }

        bool fcl_collision_checker_t::in_collision(fcl_info_t* info1, fcl_info_t* info2)
        {
            bool printing = false;
            // if(name1=="simulator/obstacles/map/point_cloud" || name2 =="simulator/obstacles/map/point_cloud")
            // {
            //     printing = true;
            // }

            if( info1->model!=NULL && info2->model!=NULL && info1->system->is_active() && info2->system->is_active() )
            {
                double dx = info1->translation[0] - info2->translation[0];
                double dy = info1->translation[1] - info2->translation[1];
                double dz = info1->translation[2] - info2->translation[2];
                double dr = info1->broadphase_radius + info2->broadphase_radius;
                // dr=PRX_INFINITY;
                // if(info1->broadphase_radius==PRX_INFINITY || info2->broadphase_radius==PRX_INFINITY )
                // {
                //     PRX_PRINT("Not created model",PRX_TEXT_RED);
                // }
                if( dx * dx + dy * dy + dz * dz < dr * dr )
                {
                    if(printing)
                    {
                        PRX_WARN_S("Actually call collision checking");
                        // PRX_INFO_S(name1<<" "<<name2);
                        PRX_INFO_S(info1->translation[0]<<" "<<info1->translation[1]<<" "<<info1->translation[2]);
                        PRX_INFO_S(info2->translation[0]<<" "<<info2->translation[1]<<" "<<info2->translation[2]);
                        PRX_WARN_S("------");
                        PRX_INFO_S(info1->quaternion.getX()<<" "<<info1->quaternion.getY()<<" "<<info1->quaternion.getZ());
                        PRX_INFO_S(info2->quaternion.getX()<<" "<<info2->quaternion.getY()<<" "<<info2->quaternion.getZ());
                    }
                    //PRX_PRINT("Not skipping collision checking.",PRX_TEXT_RED);

                    
                    //                contacts.clear();
                    //                bool valid;
                    //                valid = fcl::collide(info1->model, info1->transform, info2->model, info2->transform, 1, false, false, contacts);
                    //                fcl::CollisionObject c1(info1->model, info1->transform);
                    //                fcl::CollisionObject c2(info2->model, info2->transform);
                    fcl::CollisionRequest request;
                    fcl::CollisionResult result;
                    if( fcl::collide(info1->model, info1->transform, info2->model, info2->transform, request, result) > 0 )
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                    //(fcl::collide(&c1,&c2,request,result)>0);
                    //return result.isCollision();
                }
                else
                {
                    //PRX_PRINT("Skipping collision checkinf.", PRX_TEXT_GREEN);
                }
            }
// #endif

            return false;

        }

        bool fcl_collision_checker_t::in_collision(const std::string& name1, const std::string& name2)
        {

// #ifdef FCL_FOUND
#ifndef NDEBUG
            if( models_map[name1] == NULL )
                throw invalid_system_pair("The system " + name1 + " does not exist in the collision_checker");
            if( models_map[name2] == NULL )
                throw invalid_system_pair("The system " + name2 + " does not exist in the collision_checker");
#endif 
            fcl_info_t* info1 = models_map[name1];
            fcl_info_t* info2 = models_map[name2];
            return in_collision(info1,info2);
        }

// #ifdef FCL_FOUND
        void fcl_collision_checker_t::update_model(const std::string& name, sensor_msgs::PointCloud2& cloud, std::vector<plant_t*>& plants)
        {
            geom_map_t geom_map;
            config_list_t config_list;
            unsigned index = 0;
            models_map[name]->system->update_phys_configs(config_list,index);
            config_t cloud_conf = config_list[0].second;
            index = 0;

            foreach(plant_t* plant, plants)
            {
                if(plant->is_active())
                {
                    plant->update_phys_geoms(geom_map);
                    plant->update_phys_configs(config_list,index);
                }
            }
            // for(unsigned i=0;i<index;i++)
            // {
            //     config_list[i].second.transform_config(cloud_inverse);
            // }

            boost::shared_ptr<octomap::OcTree> tree(new octomap::OcTree(octomap_resolution));
            octomap::Pointcloud octomapCloud;
            octomap::point3d origin(0,0,0);
            sensor_msgs::PointCloud cloud1;
            sensor_msgs::convertPointCloud2ToPointCloud(cloud,cloud1);

            unsigned point_size = cloud1.points.size();
            // PRX_INFO_S("NUMBER OF POINTS: "<<point_size);  
            double x,y,z;
            vector_t point(3);
            // foreach(std::string name, geom_map | boost::adaptors::map_keys)
            // {

            if(index>0)
            {
                PRX_PRINT("Removing object from point cloud",PRX_TEXT_GREEN);
                config_t transformed_object;
                std::string object_name = config_list[0].first;
                geometry_t& obj_geom = geom_map[object_name];
                bool finished = false;
                for(unsigned i=0;i<index && !finished;i++)
                {
                    if(config_list[i].first==object_name)
                    {
                        transformed_object = config_list[i].second;
                        finished = true;
                    }
                }
                // PRX_INFO_S(obj_geom.get_type());
                // const std::vector<double>* info = obj_geom.get_info();
                // PRX_INFO_S((*info)[0]<<" "<<(*info)[1]<<" "<<(*info)[2]);
                for (unsigned i=0;i<point_size;i++)
                {
                    // Check if the point is invalid
                    if(!std::isnan(cloud1.points[i].x) && !std::isnan(cloud1.points[i].y) && !std::isnan(cloud1.points[i].z))
                    {
                        config_t obj_config;
                        obj_config.set_position(cloud1.points[i].x, cloud1.points[i].y, cloud1.points[i].z);
                        obj_config.relative_to_global(cloud_conf);
                        obj_config.global_to_relative(transformed_object);
                        obj_config.get_position(x,y,z);
                        point[0] = x;
                        point[1] = y;
                        point[2] = z;
                        
                        // if(x>-.075 && x<.075)
                        // {
                        //     PRX_INFO_S("passed x");
                        //     if(y>-.075 && y<.075)
                        //     {
                        //         PRX_INFO_S("passed y");
                        //         if(z>-.075 && z<.075)
                        //             PRX_INFO_S("passed z: should be removed");
                        //     }
                        // }
                        if(!obj_geom.contains_point(point,10*octomap_resolution))
                        {
                            octomapCloud.push_back(cloud1.points[i].x,cloud1.points[i].y,cloud1.points[i].z);
                        }
                        // else
                        // {
                        //     PRX_INFO_S("Removing points!!!!!!!!!!!!!!");
                        // }
                    }
                }
            }
            else
            {
                PRX_WARN_S("Not removing object from point cloud");
                for (unsigned i=0;i<point_size;i++)
                {
                    // Check if the point is invalid
                    if(!std::isnan(cloud1.points[i].x) && !std::isnan(cloud1.points[i].y) && !std::isnan(cloud1.points[i].z))
                    {
                        octomapCloud.push_back(cloud1.points[i].x,cloud1.points[i].y,cloud1.points[i].z);
                    }
                }

            }
            tree->insertScan(octomapCloud,origin);

            if(models_map[name]->model!=NULL)
            {
                delete models_map[name]->model;
            }
            models_map[name]->model = new fcl::OcTree(tree);
            models_map[name]->model->computeLocalAABB();


            if(name=="simulator/obstacles/map/point_cloud" && models_map[name]->model!=NULL)
            {
                PRX_INFO_S("Publishing the point cloud from PRACSYS");

                visualization_msgs::Marker marker;
                marker.header.frame_id = "/base_link";
                marker.frame_locked = false;
                marker.header.stamp = ros::Time();
                marker.ns = ros::this_node::getName();
                // marker.frame_locked = true;
                marker.id = -1;
                marker.action = visualization_msgs::Marker::ADD;
                // const vector_t& v = config.get_position();
                // const quaternion_t& q= config.get_orientation();


                marker.pose.position.x = models_map[name]->pos[0];
                marker.pose.position.y = models_map[name]->pos[1];
                marker.pose.position.z = models_map[name]->pos[2];
                marker.pose.orientation.x = models_map[name]->q.get_x();
                marker.pose.orientation.y = models_map[name]->q.get_y();
                marker.pose.orientation.z = models_map[name]->q.get_z();
                marker.pose.orientation.w = models_map[name]->q.get_w();
                // marker.pose.position.x = 0;
                // marker.pose.position.y = 0;
                // marker.pose.position.z = 0;
                // marker.pose.orientation.x = 0;
                // marker.pose.orientation.y = 0;
                // marker.pose.orientation.z = 0;
                // marker.pose.orientation.w = 1;
                marker.type = visualization_msgs::Marker::CUBE_LIST;
                marker.scale.x = octomap_resolution;
                marker.scale.y = octomap_resolution;
                marker.scale.z = octomap_resolution;
                marker.color.a = 1;
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1;

                std::vector<boost::array<fcl::FCL_REAL, 6> > boxes_ = ((fcl::OcTree*)models_map[name]->model)->toBoxes();

                for(std::size_t i = 0; i < boxes_.size(); ++i)
                {
                    fcl::FCL_REAL x = boxes_[i][0];
                    fcl::FCL_REAL y = boxes_[i][1];
                    fcl::FCL_REAL z = boxes_[i][2];
                    // PRX_INFO_S("--- "<<x<<" "<<y<<" "<<z);
                    geometry_msgs::Point point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    marker.points.push_back(point);
                }
                vis_array_pub.publish(marker);
                PRX_INFO_S("Done publishing the point cloud from PRACSYS");
                sleep(2);
            }


        }
// #endif

        std::ostream& operator<<(std::ostream& output, const fcl_collision_checker_t& checker)
        {
            output << "Systems in the collision_checker" << std::endl;
            for( hash_t<std::string, fcl_info_t*>::const_iterator iter = checker.models_map.begin(); iter != checker.models_map.end(); ++iter )
                output << iter->first << std::endl;

            output << "\nWhite list for collisions: " << std::endl;


            if( checker.collision_list )
                checker.collision_list->print();

            output << std::endl;

            return output;
        }

    }
}
