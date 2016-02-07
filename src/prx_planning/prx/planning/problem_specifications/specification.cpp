/**
 * @file specification.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/problem_specifications/specification.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/heuristic_search/astar_module.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"


namespace prx
{
    using namespace util;
    namespace plan
    {

        specification_t::specification_t()
        {
            state_space = NULL;
            control_space = NULL;
            stopping_criteria = NULL;
            astar = NULL;
            with_statistics = false;
        }

        specification_t::~specification_t()
        {
            clear();
            delete stopping_criteria;
        }

        void specification_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
        {

            // TODO: Do not use the template reader. Make your own template reader for each module!
            if( parameters::has_attribute("validity_checker", reader, template_reader) )
            {
                validity_checker = parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "validity_checker", template_reader, "validity_checker");
            }
            else
            {
                PRX_WARN_S("Missing validity_checker attribute in planning specification!");
            }


            if( parameters::has_attribute("sampler", reader, template_reader) )
            {
                sampler = parameters::initialize_from_loader<sampler_t > ("prx_planning", reader, "sampler", template_reader, "sampler");
            }
            else
            {
                PRX_WARN_S("Missing sampler attribute in planning specification!");
            }

            if( parameters::has_attribute("local_planner", reader, template_reader) )
            {
                local_planner = parameters::initialize_from_loader<local_planner_t > ("prx_planning", reader, "local_planner", template_reader, "local_planner");
            }
            else
            {
                PRX_WARN_S("Missing local_planner attribute in planning specification!");
            }

            if( parameters::has_attribute("distance_metric", reader, template_reader) )
            {
                metric = parameters::initialize_from_loader<distance_metric_t > ("prx_utilities", reader, "distance_metric", template_reader, "distance_metric");
            }
            else
            {
                PRX_WARN_S("Missing distance_metric attribute in planning specification!");
            }
            
            if( parameters::has_attribute("heuristic_search", reader, template_reader) )
            {
                astar = parameters::initialize_from_loader<astar_module_t > ("prx_planning", reader, "heuristic_search", template_reader, "heuristic_search");
            }
            else
            {
                PRX_WARN_S("Missing A*, heuristic search algorithm in planning specification!");
            }

            if( parameters::has_attribute("stopping_criteria", reader, template_reader) )
            {
                //problem with initialization(namespace)
                stopping_criteria = new stopping_criteria_t();

                std::string template_name;
                const parameter_reader_t* child_template_reader = NULL;

                if( reader->has_attribute("template") )
                {
                    template_name = reader->get_attribute("template");
                    child_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);
                }

                parameters::initialize(stopping_criteria, reader, "stopping_criteria", child_template_reader, "stopping_criteria");

                if( child_template_reader != NULL )
                {
                    delete child_template_reader;
                }
            }
            else
            {
                PRX_ERROR_S("Missing stopping criterion attribute from problem specification.");
            }

            with_statistics = parameters::get_attribute_as<bool>("with_statistics", reader, template_reader, false);
        }

        void specification_t::setup(world_model_t * const model)
        {

            if( validity_checker != NULL )
            {
                validity_checker->link_model(model);
            }
            
            if( metric != NULL )
            {
                metric->link_space(state_space);
                if(validity_checker != NULL)
                {
                    validity_checker->link_distance_function(metric->distance_function);
                }
            }
            
            if(local_planner != NULL)
            {
                local_planner->link_model(model);
                local_planner->link_sampler(sampler);
                local_planner->link_metric(metric);
                
            }
        }

        void specification_t::clear()
        {
            stopping_criteria->reset();
        }

        stopping_criteria_t* specification_t::get_stopping_criterion() const
        {
            return stopping_criteria;
        }

        void specification_t::link_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space)
        {
            this->state_space = new_state_space;
            this->control_space = new_control_space;
        }

        void specification_t::set_stopping_criterion(stopping_criteria_t* criterion)
        {
            PRX_ASSERT(criterion != NULL);
            if( stopping_criteria != NULL )
            {
                //        PRX_INFO_S("Deleting the old stopping criterion in the planning query in order to replace it with a new");
                delete stopping_criteria;
            }
            stopping_criteria = criterion;
        }

        void specification_t::gather_statistics(bool flag)
        {
            with_statistics = flag;
        }

        pluginlib::ClassLoader<specification_t> specification_t::loader("prx_planning", "prx::plan::specification_t");

        pluginlib::ClassLoader<specification_t>& specification_t::get_loader()
        {
            return loader;
        }


    }
}