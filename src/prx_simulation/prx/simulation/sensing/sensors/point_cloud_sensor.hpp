/**
 * @file point_cloud_sensor.hpp 
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
#pragma once

#ifndef PRX_POINT_CLOUD_SENSOR_HPP
#define PRX_POINT_CLOUD_SENSOR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/sensing/sensor.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/collision_checking/fcl_collision_checker.hpp"
#include "prx/simulation/system_graph.hpp"

// #ifdef FCL_FOUND
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include "prx_decision_making/DecisionMakingStateMessage.h"
#include "prx_decision_making/decision_making.h"
// #endif

namespace prx
{
	namespace util
	{
		class parameter_reader_t;
	}
	namespace sim
	{
        extern bool update_point_cloud;
        class point_cloud_sensor_t : public sensor_t
        {
            protected:
                util::geom_map_t geom_map;
                util::geom_map_t movable_bodies_geom_map;
                fcl_collision_checker_t* collision_checker;
                std::vector<plant_t*> movable_bodies;
                system_ptr_t obstacle;
                system_graph_t sys_graph;
                std::string obstacle_geometry_name;
                std::string geometry;
                std::string topic_name;

// #ifdef FCL_FOUND
                sensor_msgs::PointCloud2 cloud_in;
// #endif
                bool updated;
                ros::NodeHandle n;
                ros::Subscriber sub;
                ros::Subscriber dec_sub;
                tf::TransformListener listener;
                simulator_t* simulator;

                std::string camera_frame;
                bool left_hand;
                bool mapping_mode;

            public:

                point_cloud_sensor_t();
                virtual ~point_cloud_sensor_t();

                /**
                 * Initializes from the given parameters.
                 * 
                 * @brief Initializes from the given parameters.
                 * 
                 * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this system.
                 * @param template_reader A template \ref util::parameter_reader_t with a dictionary of parameters.
                 * Default value for the template reader is NULL. The priority goes to primary reader \c reader, if something
                 * is not specified in the \c reader then it will be read from the \c template_reader 
                 */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);
                virtual void initialize_sensor(simulator_t* sim);

                /**
                 * This function changes the point cloud geometry in the stored obstacle to the point cloud from the topic.
                 * 
                 */
                virtual void update_data(); // updates the internal sensing data (communication)

// #ifdef FCL_FOUND
                void point_cloud_callback(const sensor_msgs::PointCloud2& in_msg);

                void decision_making_state_handler(prx_decision_making::DecisionMakingStateMessagePtr system_state);
// #endif

        };
	}
}

#endif