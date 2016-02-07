/**
 * @file ui_topic.cpp 
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


#include "prx/visualization/ui_topic.hpp"

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

ui_topic_t::ui_topic_t()
{
    key_topic = node.advertise<std_msgs::Int32>("visualization/keys", 1);
    point_topic = node.advertise<geometry_msgs::Point>("visualization/points", 10);
    selected_topic = node.advertise<std_msgs::String>("visualization/selected", 10);
    camera_topic = node.advertise<geometry_msgs::PoseArray>("visualization/camera",10);
}


void ui_topic_t::keyboard( int input )
{
    std_msgs::Int32 msg;
    msg.data = (unsigned int) input;
    key_topic.publish(msg);
}

void ui_topic_t::point( const vector_t& point )
{
    geometry_msgs::Point msg;
    msg.x = point[0];
    msg.y = point[1];
    msg.z = point[2];
    point_topic.publish(msg);
}
void ui_topic_t::camera( const vector_t& camera, const vector_t& eye  )
{
    geometry_msgs::PoseArray msg;
    geometry_msgs::Pose cam_pos, eye_pos;
    cam_pos.position.x = camera[0];
    cam_pos.position.y = camera[1];
    cam_pos.position.z = camera[2];
    eye_pos.position.x = eye[0];
    eye_pos.position.y = eye[1];
    eye_pos.position.z = eye[2];
    msg.poses.push_back(cam_pos);
    msg.poses.push_back(eye_pos);
    camera_topic.publish(msg);

}

void ui_topic_t::pick( const std::string& name )
{
    PRX_DEBUG_S("Picked name to be sent: "<< name.c_str());
    std_msgs::String msg;
    msg.data = name;
    selected_topic.publish(msg);
}

    }
 }