/**
 * @file multi_planner_query.cpp
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

#include "prx/planning/task_planners/multi_planner/multi_planner_query.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"


#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS( prx::plan::multi_planner_query_t, prx::plan::query_t)


namespace prx
{
    using namespace util;
    namespace plan
    {

        void multi_planner_query_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            
            //read all the motion planning queries
            parameter_reader_t::reader_map_t query_reader = reader->get_map("planner_queries");

            const parameter_reader_t* child_template_reader = NULL;
            std::string template_name;

            foreach(const parameter_reader_t::reader_map_t::value_type query_item, query_reader)
            {
                
                if( query_item.second->has_attribute("template") )
                {
                    template_name = query_item.second->get_attribute("template");
                    child_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);
                }

                std::string plugin_type_name;

                if( child_template_reader != NULL )
                {
                    plugin_type_name = child_template_reader->get_attribute("type");
                }
                else
                {
                    plugin_type_name = query_item.second->get_attribute("type");
                }

                query_t* query = query_t::get_loader().createUnmanagedInstance("prx_planning/" + plugin_type_name);
                query->init(query_item.second, child_template_reader);


                if( child_template_reader != NULL )
                {
                    delete child_template_reader;
                    child_template_reader = NULL;
                }
                PRX_WARN_S(query_item.first);
                planner_queries[query_item.first] = query;

            }
        }

    }
}