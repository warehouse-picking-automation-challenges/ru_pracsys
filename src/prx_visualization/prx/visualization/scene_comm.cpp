/**
 * @file scene_comm.cpp 
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


#include "prx/visualization/scene_comm.hpp"
#include "prx/utilities/math/3d_geometry/geometry_info.hpp"
#include "prx/visualization/visualization.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_scene.hpp"

namespace prx
{
    using namespace util;
    namespace vis
    {

        scene_comm_t::scene_comm_t()
        {
            this->visualize_text_topic = node.subscribe("visualization/scene_text", 10, &scene_comm_t::text_callback, this);
        }

        void scene_comm_t::link_scene(scene_t* in_scene)
        {
            scene = in_scene;
        }

        /**
         *
         */
        geometry_info_t rigidBodyInfo_to_rigid_body_id(const rigid_body_info_msg* geometry)
        {
            geometry_info_t result;
            result.full_name = geometry->system_name + "/" + geometry->rigid_body_name;

            result.type = (geometry_type)geometry->geometry_type;
            result.params = geometry->geometry_params;

            result.geometry = new geometry_t(result.type, &geometry->geometry_params);
            result.uses_geom_color = false;
            if( !geometry->color_rgba.empty() )
            {
                result.uses_geom_color = true;
                result.geometry->set_color(geometry->color_rgba[0], geometry->color_rgba[1],
                                           geometry->color_rgba[2], geometry->color_rgba[3]);
            }
            else
            {
                result.geometry->set_color(-1.0, -1.0, -1.0, -1.0);
            }
            result.material_name = geometry->material_name;

            return result;
        }

        /**
         *
         */
        std::vector<geometry_info_t> describeGeometries_to_geometrymap(const describe_geometries_srv::Request* msg)
        {
            // Reserve enough space for all the geometries.
            std::vector<geometry_info_t> geometry_map;
            geometry_map.reserve(msg->rigid_body_array.size());

            // Convert RigidBodyInfo[] to geometry_map...

            foreach(rigid_body_info_msg geometry, msg->rigid_body_array)
            {
                geometry_map.push_back(rigidBodyInfo_to_rigid_body_id(&geometry));
            }

            return geometry_map;
        }

        ///**
        // *
        // */
        //class update_configurations_callback
        //{
        //private:
        //    const std::string name;
        //    const scene_comm_t* comm;
        //
        //public:
        //    update_configurations_callback(const std::string& name, 
        //                                    const scene_comm_t* incom) :
        //                                    name(name) {comm = incom;}
        //
        //    void operator()(const geometry_msgs::PoseArrayConstPtr& msg)
        //    {
        //            comm->get_scene()->move_geometry_batch(name, comm->poseArray_to_config_ts( msg ) );
        //    }
        //};

        /**
         *
         */
        bool scene_comm_t::geometry_callback(describe_geometries_srv::Request& request,
                                             describe_geometries_srv::Response& response)
        {
            PRX_DEBUG_S("CALL the geometry Callback");

            // goes into geometry service call back in visualization
            std::vector<geometry_info_t> geom_map = describeGeometries_to_geometrymap(&request);

            //    foreach(geometry_info_t g, geom_map)
            //    {
            //        g.print();
            //    }
            scene->add_geometry_batch(geom_map);

            return true;
        }

        bool scene_comm_t::plants_callback(send_plants_srv::Request& request,
                                           send_plants_srv::Response& response)
        {
            //    PRX_LOG_DEBUG ("Initialize plants callback");
            scene->initialize_plants(request.source_node_name, request.paths, request.system_path);
            return true;
        }

        bool scene_comm_t::remove_plant_callback(remove_plant_srv::Request& request,
                                                 remove_plant_srv::Response& response)
        {
            scene->remove_plant(request.path);
            return true;
        }

        bool scene_comm_t::visualize_plant_callback(visualize_plant_srv::Request& request,
                                                    visualize_plant_srv::Response& response)
        {
            PRX_DEBUG_S("Visualize plant callback");
            scene->visualize_plant(request.path, request.flag);
            return true;
        }
        
        bool scene_comm_t::visualize_obstacles_callback(visualize_obstacles_srv::Request& request,
                                                      visualize_obstacles_srv::Response& response)
        {
            PRX_ERROR_S("Visualize obstacles callback");
            scene->visualize_obstacles(request.obstacles_path);
            return true;
        }
        
        bool scene_comm_t::visualize_ghost_plants_callback(visualize_ghost_plants_srv::Request& request,
                                                  visualize_ghost_plants_srv::Response& response)
        {
            PRX_DEBUG_COLOR("Visualize ghost plants callback", PRX_TEXT_MAGENTA);
            std::vector<config_t> ghost_configs;
            for (unsigned i = 0; i < request.ghost_configs.size(); i++)
            {
                ghost_configs.push_back(pose_to_config_t(&request.ghost_configs[i]));
            }
            scene->visualize_ghost_plants(request.plant_paths, ghost_configs);
            return true;
        }
        
        void scene_comm_t::text_callback(const prx_utilities::scene_text_msg& msg)
        {
            PRX_DEBUG_S("Text callback!");
            scene_text_t new_text(msg.text_name, msg.anchored_system, msg.text, msg.font, msg.relative_position, msg.color_rgba, msg.font_size);

            scene->draw_text(new_text);
        }

    }
}
