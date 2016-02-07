/**
 * @file heuristic_task_specification.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/heuristic_task_specification.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/spaces/mapping_functions/mapping_function.hpp"

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::isst::heuristic_task_specification_t, prx::plan::specification_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace isst
        {
            heuristic_task_specification_t::heuristic_task_specification_t()
            {
                validity_checker = NULL;
                sampler = NULL;
                metric = NULL;
                local_planner = NULL;
            }

            heuristic_task_specification_t::~heuristic_task_specification_t()
            {
                clear();
                delete heuristic_specification;
                delete real_specification;
                delete state_mapping;
            }

            void heuristic_task_specification_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                specification_t::init(reader, template_reader);  
                PRX_INFO_S("Trying to create a mapping from: "<<reader->trace());
                state_mapping = parameters::create_from_loader<mapping_function_t > ("prx_utilities", reader, "state_mapping", template_reader, "state_mapping");
                PRX_INFO_S(state_mapping->mapping_name);
                heuristic_specification = dynamic_cast<motion_planning_specification_t*>(parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "heuristic_specification", template_reader, "heuristic_specification"));
                real_specification = dynamic_cast<motion_planning_specification_t*>(parameters::initialize_from_loader<specification_t > ("prx_planning", reader, "real_specification", template_reader, "real_specification"));
                    
            }
            void heuristic_task_specification_t::link_spaces(const space_t* new_state_space, const space_t* new_control_space)
            {
                // PRX_ERROR_S("Calling link_spaces on heuristic_task_specification's is incorrect. Call link_spaces on real_specification and heuristic_specification members.");
            }
            void heuristic_task_specification_t::setup(world_model_t * const model)
            {
                // heuristic_specification->setup(model);
                // real_specification->setup(model);
                // PRX_ERROR_S("Calling setup on heuristic_task_specification's is incorrect. Call setup on real_specification and heuristic_specification members.");
            }
        }
    }
}
