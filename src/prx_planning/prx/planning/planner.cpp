/**
 * @file planner.cpp 
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
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/planning/planner.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

namespace prx
{
    using namespace util;
    using namespace util::parameters;
    using namespace sim;

    namespace plan
    {

        pluginlib::ClassLoader<planner_t> planner_t::loader("prx_planning", "prx::plan::planner_t");

        pluginlib::ClassLoader<planner_t>& planner_t::get_loader()
        {
            return loader;
        }

        planner_t::planner_t()
        {
            validity_checker = NULL;
            sampler = NULL;
            local_planner = NULL;
            metric = NULL;
            path = "";
        }

        planner_t::~planner_t()
        {
            if( validity_checker != NULL )
                delete validity_checker;
            if( sampler != NULL )
                delete sampler;
            if( local_planner != NULL )
                delete local_planner;
            if( metric != NULL )
                delete metric;
        }

        void planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {            
            visualize = get_attribute_as<bool> ("visualize", reader, template_reader, true);
        }

        void planner_t::set_pathname(std::string in_path)
        {
            path = in_path;
        }

        void planner_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            if( parameter_name == "visualize" )
            {
                visualize = boost::any_cast<bool>(value);
            }
        }

        void planner_t::update_visualization() const
        {
            //    PRX_WARN_S("Planner update_visualization: visualize: "<<visualize);
            if( visualize )
                this->update_vis_info();
        }

        void planner_t::update_vis_info() const
        {
            PRX_ERROR_S("Abstract planner does not visualize any geometries.");
        }

    }
}