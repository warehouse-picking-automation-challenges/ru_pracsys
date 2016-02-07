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
#include "prx/simulation/applications/application.hpp"

#include "prx/utilities/definitions/random.hpp"

#include <fstream>

#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>


using namespace prx::util;
using namespace prx::sim;
using namespace boost::program_options;

int main(int ac, char* av[])
{
    global_reader = NULL;
    std::string node_name;
    if (ac == 1)
    {
        PRX_FATAL_S("You must specify the simulation name in args i.e. args=simulation ");
    }
    else
    {
        node_name = av[1];
        // This skips the first 8 characters, which by default in ros is: __name:=
        std::string test = node_name.substr(0,8);
        if (test == "__name:=")
        {
            node_name = node_name.substr(8);
        }
    }
    PRX_ERROR_S ("Simulation node name : " << node_name);
    
    // Initialize ROS. Call this node "simulation" when it spawns.
    ros::init(ac, av, node_name);
    ros::NodeHandle main_node_handle;

    // Wait for parameter setting scripts to finish.
    while( ros::param::has("prx/parameter_mutex") )
    {
    }

    
    
    
    std::string init_name = "prx/initialization/";
    init_name+=node_name;
    ros::param::set(init_name,true);    
    sleep(1);
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
    else
    {
        //assume waiting on a node called visualization        
        while (ros::param::has("prx/initialization/visualization")) {}
    }
    
    
    parameter_reader_t* reader;

    reader = new parameter_reader_t(node_name);
       
    int random_seed;
    try
    {

        if(reader->has_attribute("random_seed"))
        {
            random_seed = reader->get_attribute_as<int>("random_seed");
            PRX_INFO_S ("Found a random seed in input: "<< random_seed);
            init_random(random_seed);
        }
        else
        {

//            std::srand(time(NULL));
            pid_t pid = getpid();
            unsigned int id = pid;
            random_seed = rand_r(&id);
            PRX_WARN_S ("No random seed found. Setting a truly random seed: " << random_seed);

            init_random(random_seed);
        }
    }
    catch(...)
    {
        PRX_ERROR_S ("Exception caught trying to read random seed for prx_simulation.");
    }

    if (reader->has_attribute("print_random_seed"))
    {

        if (reader->get_attribute_as<bool>("print_random_seed") == true)
        {
            std::ofstream fout;
            std::string filename = ros::this_node::getName() + "_random_seed.txt";
            PRX_PRINT ("Saving random seed: " << random_seed << " to file: " << filename, PRX_TEXT_CYAN);

            fout.open(filename.substr(1).c_str(), std::fstream::app);
            fout << random_seed << std::endl;
            fout.close();
        }
    }
    
    if( reader->has_attribute("application") )
    {
        application_t* app = reader->create_from_loader<application_t > ("application", "prx_simulation");
        app->init(reader);
        
        ros::param::del(init_name); 
        ros::Timer sim_timer = main_node_handle.createTimer(ros::Duration(simulation::simulation_step), &application_t::frame, app);
//        ros::Timer tf_timer = main_node_handle.createTimer(ros::Duration(simulation::simulation_step), &application_t::tf_broadcasting, app);
        ros::Timer comm_timer = main_node_handle.createTimer(ros::Duration(0.05), &application_t::info_broadcasting, app);
        ros::Timer geom_timer = main_node_handle.createTimer(ros::Duration(0.05), &application_t::geom_broadcasting, app);

        ros::getGlobalCallbackQueue()->callAvailable();

        // Run until the visualizer or ROS wants to stop.
        
        /** Multi threaded spinner might have a problem with tf broadcaster. Commented out for now.*/
        ros::MultiThreadedSpinner spinner(4);
        spinner.spin();
//        delete reader;
//        while( ros::ok() && app->running() )
//        {            
//            ros::getGlobalCallbackQueue()->callAvailable();
////            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(simulation::simulation_step));
////            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
//        }
        
        

    }
    else
        PRX_FATAL_S("No Application has been initialized!");
    

    return 0;
}
