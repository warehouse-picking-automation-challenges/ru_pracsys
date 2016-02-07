/**
 * @file goal.cpp
 *  * 
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
#include "prx/utilities/goals/goal.hpp"

namespace prx
{
    namespace util
    {

        pluginlib::ClassLoader<goal_t> goal_t::loader("prx_utilities", "prx::util::goal_t");

        goal_t::goal_t() { }

        goal_t::~goal_t()
        {
            clear();
        }

        void goal_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            if( parameters::has_attribute("distance_metric", reader, template_reader) )
            {
                distance_metric = parameters::initialize_from_loader<distance_metric_t > ("prx_utilities", reader, "distance_metric", template_reader, "distance_metric");

                // PRX_INFO_S("Distance metric type: " << parameters::get_attribute_as<std::string > ("distance_metric/type", reader, template_reader));
                // std::string type = parameters::get_attribute_as<std::string > ("distance_metric/type",reader,template_reader);
                // distance_metric = distance_metric_t::get_loader().createUnmanagedInstance("prx_utilities/" + type);
                // if(template_reader==NULL && reader->has_attribute("distance_metric"))
                //     distance_metric->init(reader->get_child("distance_metric").get());
                // else if(reader->has_attribute("distance_metric"))
                //     distance_metric->init(reader->get_child("distance_metric").get(),template_reader->get_child("distance_metric").get());            
                // else
                //     distance_metric->init(template_reader->get_child("distance_metric").get());
            }
            else
            {
                PRX_FATAL_S("Missing distance_metric attribute in input files.");
            }
        }
        
        void goal_t::clear()
        {
            PRX_PRINT( "Goal points size: " << goal_points.size(), PRX_TEXT_BROWN);
            PRX_PRINT( "With space: " << space, PRX_TEXT_BROWN);
            foreach(space_point_t* point, goal_points)
            {
                space->free_point(point);                
            }
            goal_points.clear();
        }

        void goal_t::link_space(const space_t* inspace)
        {
            space = inspace;
            distance_metric->link_space(inspace);
        }

        void goal_t::link_metric(distance_metric_t* inmetric)
        {
            distance_metric = inmetric;
        }

        const std::vector<space_point_t*>& goal_t::get_goal_points() const
        {
            return goal_points;
        }

        pluginlib::ClassLoader<goal_t>& goal_t::get_loader()
        {
            return loader;
        }

    }
}
