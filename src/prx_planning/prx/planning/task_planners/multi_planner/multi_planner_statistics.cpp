/**
 * @file multi_planner_statistics.cpp
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

#include "prx/planning/task_planners/multi_planner/multi_planner_statistics.hpp"
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/adaptor/map.hpp>
#include <iostream>
#include <fstream>
#include <ros/ros.h>

namespace prx 
{ 
    using namespace util;
    namespace plan 
    {

multi_planner_statistics_t::multi_planner_statistics_t() 
{
    char* w = std::getenv("PRACSYS_PATH");
    std::string dir(w);
    dir += ("/prx_output/multi_planner_output/");
    boost::filesystem::path output_dir (dir);
    if (!boost::filesystem::exists(output_dir))
    {
        boost::filesystem::create_directory( output_dir );
    }
    output_directory = dir;
}

multi_planner_statistics_t::~multi_planner_statistics_t() { }

std::string multi_planner_statistics_t::get_statistics() const
{
    return "";
}

void multi_planner_statistics_t::output_stats()
{
    foreach(std::string planner_name, planner_statistics | boost::adaptors::map_keys)
    {
        if(output_check.find(planner_name)==output_check.end())
        {
            std::string filename = output_directory+ros::this_node::getName()+"_";
            filename += planner_name+".txt";
            std::ofstream fout;
            fout.open(filename.c_str());
            foreach(const statistics_t* stat, planner_statistics[planner_name])
            {
                stat->serialize(fout);
            }
            fout.close();
            output_check[planner_name] = true;
        }
    }
}

    }
}
