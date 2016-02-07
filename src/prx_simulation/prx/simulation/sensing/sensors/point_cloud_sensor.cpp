/**
 * @file point_cloud_sensor.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/sensing/sensors/point_cloud_sensor.hpp"
#include "prx/simulation/systems/obstacle.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include "simulation/plants/movable_body_plant.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp> //adaptors


PLUGINLIB_EXPORT_CLASS(prx::sim::point_cloud_sensor_t, prx::sim::sensor_t);

namespace prx
{
    using namespace util;
    
    namespace sim
    {
        bool update_point_cloud;
        point_cloud_sensor_t::point_cloud_sensor_t()
        {
            updated = false;
        }

        point_cloud_sensor_t::~point_cloud_sensor_t()
        {
        }
        
        void point_cloud_sensor_t::init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader)
        {
            sensor_t::init(reader, template_reader);
            obstacle_geometry_name = parameters::get_attribute_as<std::string>("obstacle_geometry", reader, template_reader);  
            topic_name = parameters::get_attribute_as<std::string>("topic_name", reader, template_reader);
        }
        void point_cloud_sensor_t::initialize_sensor(simulator_t* sim)
        {
            camera_frame = "/camera_rgb_optical_frame";
            mapping_mode = false;
            left_hand = true;
            std::string nm;
            boost::tie(nm, geometry) = reverse_split_path(obstacle_geometry_name);

            hash_t<std::string, system_ptr_t>& obstacles = sim->get_obstacles();

            foreach(system_ptr_t ptr, obstacles | boost::adaptors::map_values)
            {
                std::vector<std::string>* names = static_cast<obstacle_t*>(ptr.get())->get_geometries_names();
                foreach(std::string& name, *names)
                {
                    // PRX_INFO_S(name);
                    if(name.compare(obstacle_geometry_name)==0)
                    {
                        obstacle = ptr;
                    }
                }
            }

            collision_checker = dynamic_cast<fcl_collision_checker_t*>(sim->get_collision_checker());
            if(collision_checker==NULL)
            {
                PRX_FATAL_S("Point Cloud collision checks are only available with FCL collision checking.");
            }

// #ifdef FCL_FOUND
            std::string temp_topic = "/decision_making_state";
            sub = n.subscribe(topic_name,1,&point_cloud_sensor_t::point_cloud_callback,this);
            dec_sub = n.subscribe(temp_topic, 1, &point_cloud_sensor_t::decision_making_state_handler,this);
// #endif
            obstacle->update_phys_geoms(geom_map);
            sim->update_phys_geoms(movable_bodies_geom_map);
            sim->update_system_graph(sys_graph);
            std::vector<plant_t*> plants;
            sys_graph.get_plants(plants);
            foreach(plant_t* plant, plants)
            {
                packages::manipulation::movable_body_plant_t* p = dynamic_cast<packages::manipulation::movable_body_plant_t*>(plant);
                if(p!=NULL)
                {
                    movable_bodies.push_back(plant);
                }
            }
            PRX_INFO_S("Number of objects: "<<movable_bodies.size());
            simulator = sim;
        }
        
        void point_cloud_sensor_t::update_data()
        {   

// #ifdef FCL_FOUND
            if(updated && update_point_cloud)
            {
                //get root config for point cloud

                tf::StampedTransform cloud_transform;
                int iter = 0;
                while(iter++<5)
                {
                    try
                    {
                        listener.lookupTransform("/base_link", camera_frame,
                                               ros::Time(0), cloud_transform);
                    }
                    catch (tf::TransformException &ex) 
                    {
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        if(iter==5)
                        {
                            PRX_ERROR_S("Failed to get point cloud transform when computing updating point cloud.");
                        }
                    }
                }

                config_t config;
                config.set_position(cloud_transform.getOrigin().x(),cloud_transform.getOrigin().y(),cloud_transform.getOrigin().z());
                config.set_orientation(cloud_transform.getRotation().getX(),cloud_transform.getRotation().getY(),cloud_transform.getRotation().getZ(),cloud_transform.getRotation().getW());
                PRX_INFO_S("config of point cloud: "<<config);
                static_cast<obstacle_t*>(obstacle.get())->update_root_configuration(config);
                simulator->update_obstacles_in_collision_checker();
                collision_checker->update_model(obstacle_geometry_name,cloud_in,movable_bodies);

                // updated = false;
            }
// #endif
        }

// #ifdef FCL_FOUND
        void point_cloud_sensor_t::point_cloud_callback(const sensor_msgs::PointCloud2& in_msg)
        {
            if(mapping_mode)
            {
                cloud_in = in_msg;
                updated = true;
                mapping_mode = false;
            }
        }

        void point_cloud_sensor_t::decision_making_state_handler(prx_decision_making::DecisionMakingStateMessagePtr system_state)
        {
            if(system_state->state == Map_and_Detect_Object)
            {
                mapping_mode = true;
            }
            else
            {
                mapping_mode = false;
            }

            left_hand = system_state->robot_arm==0;
            if(left_hand)
                camera_frame =  "/camera_rgb_optical_frame";
            else
                camera_frame =  "/camera2_rgb_optical_frame";

        }
// #endif
    }
}