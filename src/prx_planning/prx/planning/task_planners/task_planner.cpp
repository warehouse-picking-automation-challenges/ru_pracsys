/**
 * @file task_planner.cpp 
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

#include "prx/planning/task_planners/task_planner.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/planning/queries/query.hpp"
#include "prx/planning/problem_specifications/specification.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include <boost/range/adaptor/map.hpp> //adaptors

namespace prx
{
    using namespace util;
    namespace plan
    {

        task_planner_t::task_planner_t()
        {
            model = NULL;
            path = "task_planner";
        }

        task_planner_t::~task_planner_t()
        {

            foreach(planner_t* planner, planners | boost::adaptors::map_values)
            {
                delete planner;
            }
            
            foreach(specification_t* specification, output_specifications | boost::adaptors::map_values)
            {
                delete specification;
            }

            foreach(query_t* query, output_queries | boost::adaptors::map_values)
            {
                delete query;
            }
        }

        void task_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            PRX_DEBUG_S("Inside init of task planner with name "<<path);
            std::string template_name;
            planner_t::init(reader, template_reader);
            
            //initialize the motion planners
            if( reader->has_attribute("planners") )
            {
                parameter_reader_t::reader_map_t planner_map = reader->get_map("planners");

                foreach(const parameter_reader_t::reader_map_t::value_type key_value, planner_map)
                {
                    PRX_INFO_S("Creating a planner with name: " << key_value.first);
                    const parameter_reader_t* child_template_reader = NULL;

                    if( key_value.second->has_attribute("template") )
                    {
                        template_name = key_value.second->get_attribute("template");
                        child_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);
                    }
                    planner_t* new_planner = parameters::create_from_loader<planner_t > ("prx_planning", key_value.second, "", child_template_reader, "");
                    std::string planner_name = key_value.first;
                    new_planner->set_pathname(path + "/" + planner_name);
                    parameters::initialize(new_planner,key_value.second,"",child_template_reader,"");
                    //perform an operation to get the mappings from the world model and pass them to the motion planners
                    planner_names.push_back(planner_name);
                    planners[planner_name] = new_planner;
                    planners[planner_name]->set_pathname(path + "/" + planner_name);

                    space_names[planner_name] = parameters::get_attribute_as<std::string > ("space_name", key_value.second, child_template_reader);

                    if( child_template_reader != NULL )
                    {
                        delete child_template_reader;
                        child_template_reader = NULL;
                    }
                }
            }     
        }
        

        void task_planner_t::set_param(const std::string& path, const std::string& parameter_name, const boost::any& value)
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            if( name.empty() )
                set_param(parameter_name, value);
            else if( planners.find(name) != planners.end() )
                planners[name]->set_param(subpath, parameter_name, value);
            else
                PRX_FATAL_S("The path to the planner was malformed. No planner with name "<<name<<" underneath task_planner "<<path);
        }

        void task_planner_t::link_world_model(world_model_t * const model)
        {
            this->model = model;

            foreach(std::string planner_name, planners | boost::adaptors::map_keys)
            {
                model->use_context(space_names[planner_name]);
                planners[planner_name]->link_world_model(model);
            }                 
        }
        
        void task_planner_t::link_specification(specification_t* new_spec)
        {
            input_specification = new_spec;
            
            if( input_specification->local_planner != NULL )
                local_planner = input_specification->local_planner;
            else
                PRX_WARN_S("No local planner has been specified in task planner " << path);
            
            if( input_specification->validity_checker != NULL )
                validity_checker = input_specification->validity_checker;
            else
                PRX_WARN_S("No validity checker has been specified in task planner " << path);

            if( input_specification->sampler != NULL )
                sampler = input_specification->sampler;
            else
                PRX_WARN_S("No sampler has been specified in task planner " << path);

            if( input_specification->metric != NULL )
                metric = input_specification->metric;
            else
                PRX_WARN_S("No distance metric has been specified in task planner " << path);
            
        }

        void task_planner_t::link_query(query_t* new_query)
        {
            input_query = new_query;            
        }

        void task_planner_t::reset()
        {

            foreach(planner_t* planner, planners | boost::adaptors::map_values)
            {
                planner->reset();
            }
        }
        
        void task_planner_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            planner_t::set_param(parameter_name, value);
        }

    }
}
