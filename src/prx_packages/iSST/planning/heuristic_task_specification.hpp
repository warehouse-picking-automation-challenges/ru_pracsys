/**
 * @file heuristic_task_specification.hpp
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
#pragma once

#ifndef PRX_HEURISTIC_TASK_SPECIFICATION_HPP
#define	PRX_HEURISTIC_TASK_SPECIFICATION_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/problem_specifications/specification.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
        class mapping_function_t;
        class distance_metric_t;
    }
    namespace plan
    {
        class motion_planning_specification_t;
        class world_model_t;
    }
    
    namespace packages
    {
        namespace isst
        {
            /**
             * @anchor heuristic_task_specification_t
             *             
             * @brief <b> Specification for heuristic task planners. </b>
             *
             * @author Zakary Littlefield
             */
            class heuristic_task_specification_t : public plan::specification_t
            {

              public:
                heuristic_task_specification_t();
                virtual ~heuristic_task_specification_t();

                /**
                 * @brief Initialize the planning specification from input parameters.
                 *
                 * Initialize the planning specification from input parameters.
                 * 
                 * @param reader The reader for the parameters.
                 * @param template_reader A template reader for reading template parameters 
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * Prepare the planning specification to be linked to the children
                 * 
                 * @brief Prepare the planning specification to be linked to the children
                 */
                virtual void setup(plan::world_model_t * const model);

                /** @copydoc specification_t::link_spaces(const util::space_t*, const util::space_t*) */
                virtual void link_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space);

                /** @brief Mapping between the heuristic state space and the real state space*/
                util::mapping_function_t* state_mapping;

                //temporary storage in the specification
                plan::motion_planning_specification_t* heuristic_specification;
                plan::motion_planning_specification_t* real_specification;
            };
        }

    }
}

#endif

