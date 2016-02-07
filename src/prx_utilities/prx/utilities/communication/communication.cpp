/**
 * @file communication.cpp 
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


#include "prx/utilities/communication/communication.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx
{
    namespace util
    {

        using namespace prx_utilities;
        static int marker_id = 0;

        visualization_msgs::Marker communication_t::send_marker(const geometry_info_t& info)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = info.full_name;
            marker.header.stamp = ros::Time();
            marker.ns = ros::this_node::getName();
            marker.frame_locked = true;
            marker.lifetime = ros::Duration();
            marker.id = marker_id++;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            //vector_t info_geom_color = info.get_color(); 
            //marker.color.r = info_geom_color[0];
            //marker.color.g = info_geom_color[1];
            //marker.color.b = info_geom_color[2];
            //marker.color.a = info_geom_color[3];

            switch(info.type)
            {
                case 1:
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.scale.x = 2*info.params[0];
                    marker.scale.y = 2*info.params[0];
                    marker.scale.z = 2*info.params[0];
                    break;
                case 2:
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.scale.x = info.params[0];
                    marker.scale.y = info.params[1];
                    marker.scale.z = info.params[2];
                    break;
                case 3:
                    PRX_FATAL_S("RViz can't visualize cones");
                    break;
                case 4:
                    marker.type = visualization_msgs::Marker::CYLINDER;
                    marker.scale.x = 2*info.params[0];
                    marker.scale.y = 2*info.params[0];
                    marker.scale.z = info.params[1];
                    break;
                case 5:
                    PRX_FATAL_S("RViz can't visualize open cylinders");
                    break;
                case 6:
                    PRX_FATAL_S("RViz can't visualize capsules");
                    break;
                case 9:
                    PRX_FATAL_S("We can't visualize lines right now");
                    break;
                case 10:
                    marker.type = visualization_msgs::Marker::LINE_STRIP;
                    marker.scale.x = .1;
                    for(unsigned i=0;i<info.params.size();i+=3)
                    {
                        geometry_msgs::Point point;
                        point.x = info.params[i];
                        point.y = info.params[i+1];
                        point.z = info.params[i+2];
                        marker.points.push_back(point);
                    }
                    break;
                case 11:
                    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                    //only if using a MESH_RESOURCE marker type:
                    marker.mesh_resource = "package://prx_models/";
                    marker.mesh_resource+=info.mesh_filename;
                    PRX_PRINT("I am about to send a MEsh1 : " << marker.mesh_resource, PRX_TEXT_BROWN);
                    marker.mesh_use_embedded_materials = true;
                    break;
                case 12:
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.scale.x = .01;
                    marker.scale.y = .01;
                    marker.scale.z = .01;
                    break;
                default:
                    PRX_FATAL_S("Invalid geometry type: " << info.type);
            }
            return marker;
        }

        visualization_msgs::Marker communication_t::send_marker(const geometry_t& info, std::string name)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = name;
            marker.header.stamp = ros::Time();
            marker.ns = ros::this_node::getName();
            marker.frame_locked = true;
            marker.lifetime = ros::Duration();
            marker.id = marker_id++;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            vector_t info_geom_color = info.get_color(); 
            marker.color.r = info_geom_color[0];
            marker.color.g = info_geom_color[1];
            marker.color.b = info_geom_color[2];
            marker.color.a = info_geom_color[3];

            switch(info.get_type())
            {
                case 1:
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.scale.x = 2*(*info.get_info())[0];
                    marker.scale.y = 2*(*info.get_info())[0];
                    marker.scale.z = 2*(*info.get_info())[0];
                    break;
                case 2:
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.scale.x = (*info.get_info())[0];
                    marker.scale.y = (*info.get_info())[1];
                    marker.scale.z = (*info.get_info())[2];
                    break;
                case 3:
                    PRX_FATAL_S("RViz can't visualize cones");
                    break;
                case 4:
                    marker.type = visualization_msgs::Marker::CYLINDER;
                    marker.scale.x = 2*(*info.get_info())[0];
                    marker.scale.y = 2*(*info.get_info())[0];
                    marker.scale.z = (*info.get_info())[1];
                    break;
                case 5:
                    PRX_FATAL_S("RViz can't visualize open cylinders");
                    break;
                case 6:
                    PRX_FATAL_S("RViz can't visualize capsules");
                    break;
                case 9:
                    PRX_FATAL_S("We can't visualize lines right now");
                    break;
                case 10:
                    marker.type = visualization_msgs::Marker::LINE_STRIP;
                    marker.scale.x = .1;
                    for(unsigned i=0;i<info.get_info()->size();i+=3)
                    {
                        geometry_msgs::Point point;
                        point.x = (*info.get_info())[i];
                        point.y = (*info.get_info())[i+1];
                        point.z = (*info.get_info())[i+2];
                        marker.points.push_back(point);
                    }
                    break;
                case 11:
                    marker.scale.x = 1.0;
                    marker.scale.y = 1.0;
                    marker.scale.z = 1.0;
   
                    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                    //only if using a MESH_RESOURCE marker type:
                    marker.mesh_resource = "package://prx_models/";
                    marker.mesh_resource+=info.get_mesh();
                    //PRX_PRINT("I am about to send a MEsh2 : " << marker.mesh_resource, PRX_TEXT_BROWN);
                    marker.mesh_use_embedded_materials = true;                    
                    break;
                case 12:
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.scale.x = .01;
                    marker.scale.y = .01;
                    marker.scale.z = .01;
                    break;
                default:
                    PRX_FATAL_S("Invalid geometry type: " << info.get_type());
            }
            return marker;
        }

        visualization_msgs::Marker communication_t::send_marker(const geometry_t& info, std::string name, const config_t& config )
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/base_link";
            marker.frame_locked = true;
            marker.header.stamp = ros::Time();
            marker.lifetime = ros::Duration();
            marker.ns = ros::this_node::getName();
            // marker.frame_locked = true;
            marker.id = marker_id++;
            marker.action = visualization_msgs::Marker::ADD;

            const vector_t& v = config.get_position();
            const quaternion_t& q= config.get_orientation();


            marker.pose.position.x = v[0];
            marker.pose.position.y = v[1];
            marker.pose.position.z = v[2];
            marker.pose.orientation.x = q.get_x();
            marker.pose.orientation.y = q.get_y();
            marker.pose.orientation.z = q.get_z();
            marker.pose.orientation.w = q.get_w();

            vector_t info_geom_color = info.get_color(); 
            marker.color.r = info_geom_color[0];
            marker.color.g = info_geom_color[1];
            marker.color.b = info_geom_color[2];
            marker.color.a = info_geom_color[3];

            switch(info.get_type())
            {
                case 1:
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.scale.x = 2*(*info.get_info())[0];
                    marker.scale.y = 2*(*info.get_info())[0];
                    marker.scale.z = 2*(*info.get_info())[0];
                    break;
                case 2:
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.scale.x = (*info.get_info())[0];
                    marker.scale.y = (*info.get_info())[1];
                    marker.scale.z = (*info.get_info())[2];
                    break;
                case 3:
                    PRX_FATAL_S("RViz can't visualize cones");
                    break;
                case 4:
                    marker.type = visualization_msgs::Marker::CYLINDER;
                    marker.scale.x = 2*(*info.get_info())[0];
                    marker.scale.y = 2*(*info.get_info())[0];
                    marker.scale.z = (*info.get_info())[1];
                    break;
                case 5:
                    PRX_FATAL_S("RViz can't visualize open cylinders");
                    break;
                case 6:
                    PRX_FATAL_S("RViz can't visualize capsules");
                    break;
                case 9:
                    PRX_FATAL_S("We can't visualize lines right now");
                    break;
                case 10:
                    marker.type = visualization_msgs::Marker::LINE_STRIP;
                    marker.scale.x = .1;
                    for(unsigned i=0;i<info.get_info()->size();i+=3)
                    {
                        geometry_msgs::Point point;
                        point.x = (*info.get_info())[i];
                        point.y = (*info.get_info())[i+1];
                        point.z = (*info.get_info())[i+2];
                        marker.points.push_back(point);
                    }
                    break;
                case 11:
                    marker.type = visualization_msgs::Marker::MESH_RESOURCE;                    
                    //only if using a MESH_RESOURCE marker type:
                    marker.mesh_resource = "package://prx_models/";
                    marker.mesh_resource+=info.get_mesh();
                    PRX_PRINT("I am about to send a MEsh : " << marker.mesh_resource, PRX_TEXT_BROWN);
                    marker.mesh_use_embedded_materials = true;
                    break;
                case 12:
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.scale.x = .01;
                    marker.scale.y = .01;
                    marker.scale.z = .01;
                    break;
                default:
                    PRX_FATAL_S("Invalid geometry type: " << info.get_type());
            }
            return marker;
        }

        rigid_body_info_msg rigidbody_to_rigidBodyInfo(const geometry_info_t& info)
        {
            rigid_body_info_msg result;

            result.system_name = info.system_name;
            result.rigid_body_name = info.rigid_body_name;
            result.geometry_type = info.type;
            result.material_name = info.material_name;
            if( info.uses_geom_color )
            {
                result.color_rgba.push_back(info.color[0]);
                result.color_rgba.push_back(info.color[1]);
                result.color_rgba.push_back(info.color[2]);
                result.color_rgba.push_back(info.color[3]);
            }
            for( size_t i = 0; i < info.params.size(); ++i )
                result.geometry_params.push_back(info.params.at(i));

            return result;
        }

        void communication_t::publish_markers()
        {
            // ros::Duration timeout(5);
            // ros::Time start = ros::Time::now();
            // while(ros::Time::now() - start < timeout) {
            //     if(vis_array_pub.getNumSubscribers() > 0)
            //         break;
            // }
            vis_array_pub.publish(array);
            array.markers.clear();
        }

        geometry_msgs::Point communication_t::vector_t_to_point(const vector_t& pos) const
        {
            geometry_msgs::Point point; // = new geometry_msgs::Point;
            point.x = pos[0];
            point.y = pos[1];
            point.z = pos[2];
            return point;
        }

        vector_t communication_t::point_to_vector_t(const geometry_msgs::Point* point) const
        {
            vector_t ret; // = new vector_t(3);
            ret[0] = point->x;
            ret[1] = point->y;
            ret[2] = point->z;
            return ret;
        }

        geometry_msgs::Quaternion communication_t::quaternion_t_to_quaternion(const quaternion_t& quat) const
        {
            geometry_msgs::Quaternion quaternion; // = new geometry_msgs::Quaternion;
            quaternion.x = quat.get_x();
            quaternion.y = quat.get_y();
            quaternion.z = quat.get_z();
            quaternion.w = quat.get_w();
            return quaternion;
        }

        quaternion_t communication_t::quaternion_to_quaternion_t(const geometry_msgs::Quaternion* quat) const
        {
            quaternion_t ret; // = new quaternion_t();
            ret.set(quat->x, quat->y, quat->z, quat->w);
            return ret;
        }

        geometry_msgs::Pose communication_t::config_t_to_pose(const config_t& config) const
        {
            geometry_msgs::Pose pose;
            const vector_t& pos = config.get_position();
            const quaternion_t& quat = config.get_orientation();

            pose.position.x = pos[0];
            pose.position.y = pos[1];
            pose.position.z = pos[2];

            pose.orientation.x = quat.get_x();
            pose.orientation.y = quat.get_y();
            pose.orientation.z = quat.get_z();
            pose.orientation.w = quat.get_w();

            return pose;
        }


        //geometry_msgs::PoseArray communication_t::config_ts_to_poseArray(const std::vector< const config_t&>& msg) const
        //{
        //    geometry_msgs::PoseArray poses;
        //
        //    PRX_LOG_ERROR("You have to re-implement the config_ts_to_poseArray");
        ////    for( unsigned int i = 0; i<msg.size(); ++i)                
        ////        poses.poses.push_back(config_t_to_pose(msg[i]));
        //
        //    return poses;
        //}
        //
        //
        //geometry_msgs::PoseArray communication_t::batch_configs_to_poseArray(const std::vector< const config_t&>& msg) const
        //{
        //    geometry_msgs::PoseArray poses;
        //
        //    PRX_LOG_ERROR("You have to re-implement the batch_configs_to_poseArray");
        ////    for( unsigned int i = 0; i<msg.size(); ++i)                
        ////        poses.poses.push_back(config_t_to_pose(msg[i]));
        //
        //    return poses;
        //}

        config_t communication_t::pose_to_config_t(geometry_msgs::Pose* pose) const
        {
            config_t config;

            config.set_position(pose->position.x, pose->position.y, pose->position.z);
            config.set_xyzw_orientation(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w);

            return config;
        }

        std::vector<config_t> communication_t::poseArray_to_config_ts(const geometry_msgs::PoseArrayConstPtr msg) const
        {
            std::vector<config_t> configs;
            configs.reserve(msg->poses.size());

            foreach(geometry_msgs::Pose pose, msg->poses)
            configs.push_back(pose_to_config_t(&pose));

            return configs;
        }

        //bool communication_t::create_geometry_message(describe_geometries_srv::Request& request, std::vector<geometry_info_t>& info)
        //{
        //    foreach(const geometry_info_t body, info)
        //        request.rigid_body_array.push_back(rigidbody_to_rigidBodyInfo(body));
        //
        //    return true;
        //}

        bool communication_t::send_extra_geometries(const hash_t<std::string, geometry_info_t>& new_geometries, double duration, const std::string& destination_node)
        {
            PRX_DEBUG_COLOR("Sending geometries", PRX_TEXT_LIGHTGRAY);
            //    PRX_DEBUG_S("The size of the geometries that I am sending is : " << new_geometries.size());
            ros::ServiceClient client = node.serviceClient<describe_geometries_srv > (destination_node + "/geometries");
            describe_geometries_srv srv;

            for( hash_t<std::string, geometry_info_t, string_hash>::const_iterator iter = new_geometries.begin(); iter != new_geometries.end(); ++iter )
            {
                visualization_msgs::Marker marker = send_marker(iter->second);
                array.markers.push_back(marker);
                // vis_pub.publish( marker );
            }
            publish_markers();

            for( hash_t<std::string, geometry_info_t, string_hash>::const_iterator iter = new_geometries.begin(); iter != new_geometries.end(); ++iter )
                srv.request.rigid_body_array.push_back(rigidbody_to_rigidBodyInfo(iter->second));
            //    foreach(const geometry_info_t body, new_geometries)
            //        srv.request.rigid_body_array.push_back(rigidbody_to_rigidBodyInfo(body));

            if( try_visualization_again )
                duration = -1;

            client.waitForExistence(ros::Duration(duration));

            if( !client.call(srv) )
            {
                if( destination_node == "visualization" )
                    try_visualization_again = false;

                PRX_WARN_S("Service request for geometries to " << destination_node << " failed.  Ignoring.");
                return false;
            }
            return true;
        }

        bool communication_t::send_plants(const std::string& source_node, const std::vector<std::string>& plant_paths, const std::vector<std::string>& template_paths, double duration, const std::string& destination_node)
        {
            // marker_id = 0;
            PRX_INFO_S("Send plants ");
            ros::ServiceClient client = node.serviceClient<send_plants_srv > (destination_node + "/plants");
            send_plants_srv srv;

            srv.request.source_node_name = source_node;
            
            foreach(std::string path, plant_paths)
            srv.request.paths.push_back(path);

            foreach(std::string path, template_paths)
            srv.request.system_path.push_back(path);

            if( try_visualization_again )
                duration = -1;

            client.waitForExistence(ros::Duration(duration));

            if( !client.call(srv) )
            {
                if( destination_node == "visualization" )
                    try_visualization_again = false;

                PRX_ERROR_S("Service request to send plants to " << destination_node << " failed.  Ignoring.");
                return false;
            }
            return true;
        }
        
        

        bool communication_t::remove_plant(const std::string& path, double duration, const std::string& destination_node)
        {
            PRX_INFO_S("Remove plant: " << path);
            ros::ServiceClient client = node.serviceClient<remove_plant_srv > (destination_node + "/remove_plant");
            remove_plant_srv srv;

            srv.request.path = path;

            if( try_visualization_again )
                duration = -1;

            client.waitForExistence(ros::Duration(duration));

            if( !client.call(srv) )
            {
                if( destination_node == "visualization" )
                    try_visualization_again = false;

                PRX_WARN_S("Service request to remove plants to " << destination_node << " failed.  Ignoring.");
                return false;
            }
            return true;
        }

        bool communication_t::visualize_plant(const std::string& path, int flag, double duration, const std::string& destination_node)
        {
            //    if (flag)
            //        PRX_INFO_S("Visualizing plant: " << path);
            //    else
            //        PRX_INFO_S("Not visualizing plant: " << path);

            ros::ServiceClient client = node.serviceClient<visualize_plant_srv > (destination_node + "/visualize_plant");
            visualize_plant_srv srv;

            srv.request.path = path;
            srv.request.flag = flag;

            if( try_visualization_again )
                duration = -1;

            client.waitForExistence(ros::Duration(duration));

            if( !client.call(srv) )
            {
                if( destination_node == "visualization" )
                    try_visualization_again = false;

                PRX_ERROR_S("Service request to visualize plants to " << destination_node << " failed!  Ignoring.");
                return false;
            }
            return true;
        }

        bool communication_t::visualize_obstacles(const std::string& path, double duration, const std::string& destination_node)
        {
            //    if (flag)
            //        PRX_INFO_S("Visualizing plant: " << path);
            //    else
            //        PRX_INFO_S("Not visualizing plant: " << path);

            ros::ServiceClient client = node.serviceClient<visualize_obstacles_srv > (destination_node + "/visualize_obstacles");
            visualize_obstacles_srv srv;

            srv.request.obstacles_path = path;

            if( try_visualization_again )
                duration = -1;

            client.waitForExistence(ros::Duration(duration));

            if( !client.call(srv) )
            {
                if( destination_node == "visualization" )
                    try_visualization_again = false;

                PRX_ERROR_S("Service request to visualize obstacles at:" << path << " to vis node: " << destination_node << " failed!  Ignoring.");
                return false;
            }
            return true;
        }
        
        bool communication_t::update_info_geoms(const std::vector<std::string>& geom_names, const std::vector<util::config_t>& geom_configs, const std::vector<util::vector_t>& geom_colors, bool poll_tf, double duration, const std::string& destination_node)
        {
            PRX_DEBUG_COLOR ("Update info geoms", PRX_TEXT_CYAN);
            ros::ServiceClient client = node.serviceClient<update_info_geoms_srv > (destination_node + "/update_info_geoms");
            update_info_geoms_srv srv;

            srv.request.info_names = geom_names;
            if (!geom_configs.empty())
            {
                foreach(config_t conf, geom_configs)
                {
                    srv.request.info_configs.push_back(config_t_to_pose(conf));
                }
            }
            if (!geom_colors.empty())
            {
                foreach(vector_t vec, geom_colors)
                {
                    Vec4_msg color_vec;
                    color_vec.r = vec[0];
                    color_vec.g = vec[1];
                    color_vec.b = vec[2];
                    color_vec.a = vec[3];
                    srv.request.info_colors.push_back(color_vec);
                }
            }
            srv.request.poll_tf = poll_tf;

            if( try_visualization_again )
                duration = -1;

            client.waitForExistence(ros::Duration(duration));

            if( !client.call(srv) )
            {
                if( destination_node == "visualization" )
                    try_visualization_again = false;

                PRX_WARN_S("Service request to update info geoms " << destination_node << " failed!  Ignoring.");
                return false;
            }
            return true;
            
        }
        
        bool communication_t::visualize_ghost_plant(const std::vector<std::string>& plant_paths, const std::vector<util::config_t>& plant_configs, double duration, const std::string& destination_node)
        {
            PRX_DEBUG_COLOR ("Visualize ghosts", PRX_TEXT_CYAN);
            ros::ServiceClient client = node.serviceClient<visualize_ghost_plants_srv > (destination_node + "/visualize_ghost_plants");
            visualize_ghost_plants_srv srv;

            srv.request.plant_paths = plant_paths;
 
            foreach(config_t conf, plant_configs)
            {
                srv.request.ghost_configs.push_back(config_t_to_pose(conf));
            }

            if( try_visualization_again )
                duration = -1;

            client.waitForExistence(ros::Duration(duration));

            if( !client.call(srv) )
            {
                if( destination_node == "visualization" )
                    try_visualization_again = false;

                PRX_WARN_S("Service request to show ghost plants to " << destination_node << " failed!  Ignoring.");
                return false;
            }
            return true;
            
        }
        bool communication_t::take_screenshot(unsigned screen_num, int number_of_screenshots, double duration, const std::string& destination_node)
        {
            PRX_INFO_S ("Taking screenshot");
            ros::ServiceClient client = node.serviceClient<take_screenshot_srv > (destination_node + "/take_screenshot");
            take_screenshot_srv srv;

            srv.request.screen_num = screen_num;
            srv.request.number_of_screenshots = number_of_screenshots;
            srv.request.destination_node = destination_node;
 
            if( try_visualization_again )
                duration = -1;

            client.waitForExistence(ros::Duration(duration));

            if( !client.call(srv) )
            {
                if( destination_node == "visualization" )
                    try_visualization_again = false;

                PRX_WARN_S("Service request to take screen shot on " << destination_node << " failed!  Ignoring.");
                return false;
            }
            return true;
            
        }
        bool communication_t::shutdown_node(const std::string& source_node, const std::string& destination_node, double duration)
        {

            PRX_INFO_S("Node: " << source_node << "Requesting Shutdown of node: " << destination_node);
            ros::ServiceClient client = node.serviceClient<shutdown_node_srv > (destination_node + "/shutdown_node");
            shutdown_node_srv srv;

            srv.request.source_node_name = source_node;
            srv.request.destination_node_name = destination_node;

            client.waitForExistence(ros::Duration(duration));

            if( !client.call(srv) )
            {
                PRX_WARN_S("Service request for node " << destination_node << " to shutdown failed.  Ignoring.");
                return false;
            }
            return true;
        }

    }
}


