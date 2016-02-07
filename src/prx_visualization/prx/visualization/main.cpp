/**
 * @file main.cpp 
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/visualization/visualization.hpp"
#include "prx/visualization/visualization_comm.hpp"
#include "prx/visualization/ui_topic.hpp"

#include <pluginlib/class_loader.h>
#include <vector>
#include <iostream>
#include <boost/bind.hpp>

using namespace prx::util;
using namespace prx::vis;


int main( int ac, char* av[] )
{
    
    global_reader = NULL;
    std::string node_name;
    if (ac == 1)
        node_name = "visualization";
    else
        node_name = av[1];
    PRX_INFO_S ("Visualization node name : " << node_name);

    // Initialize ROS. Call this node "visualization" when it spawns.
    ros::init(ac, av, "visualization");

    // Wait for parameter setting scripts to finish.
    while (ros::param::has("prx/parameter_mutex")) {}
    
    std::string init_name = "prx/initialization/";
    init_name+=node_name;
    ros::param::set(init_name,true);  
    sleep(.5);
    if (ros::param::has("prx/initialization/order"))
    {
        parameter_reader_t init_reader("prx/initialization/");
        //make sure nothing comes before this
        std::vector<std::string> node_list = init_reader.get_attribute_as<std::vector<std::string> >("order");
        unsigned pos=node_list.size();
        for(unsigned i=0;i<pos;i++)
        {
            if(node_list[i]==node_name)
                pos = i;
        }
        for(unsigned i=0;i<pos;i++)
        {
            while (ros::param::has("prx/initialization/"+node_list[i]))
            {
                sleep(1);
            }
        }
    }
    

    parameter_reader_t reader("visualization/");
    const std::string plugin_type_name = reader.get_attribute("type");

    visualization_t* vis = NULL;
    try
    {
    	pluginlib::ClassLoader<visualization_t> vis_loader("prx_visualization", "prx::vis::visualization_t");
    	vis = vis_loader.createUnmanagedInstance(plugin_type_name);
    }
    catch (pluginlib::PluginlibException& ex)
    {
	   PRX_FATAL_S("Failed to load visualization plugin: " << ex.what());
    }
    vis->init(&reader);

    // Initialize services.
    ui_topic_t* ui = new ui_topic_t();

    vis->get_viewer()->set_handler(ui);

    ros::param::del(init_name); 
    // Run until the visualizer or ROS wants to stop.
    vis->run();

    delete vis;
    return 0;
}
