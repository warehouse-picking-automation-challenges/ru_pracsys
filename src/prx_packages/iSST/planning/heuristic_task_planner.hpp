 /**
 * @file heuristic_task_planner.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once
#ifndef PRX_HEURISTIC_TASK_PLANNER_HPP
#define	PRX_HEURISTIC_TASK_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/utilities/goals/goal_state.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"


namespace prx
{    
    namespace plan
    {
        class motion_planning_query_t;
        class specification_t;
        class query_t;
        class isst_t;
        class motion_planner_t;
    }
    namespace packages
    {
        namespace isst
        {
            class heuristic_task_specification_t;
            /**
             * @class heuristic_task_planner_t
             * 
             * A task planner that uses a roadmap to generate a heuristic function for another motion planner.
             * 
             * @author Zakary Littlefield
             * 
             */
            class heuristic_task_planner_t : public plan::task_planner_t
            {  
              public:
                           
                /**
                 * @copydoc task_planner_t::init()
                 */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

                /**
                 * @copydoc task_planner_t::setup()
                 */
                virtual void setup();

                /**
                 * @copydoc task_planner_t::execute()
                 */
                virtual bool execute();

                /**
                 * @copydoc task_planner_t::get_statistics()
                 */
                virtual const util::statistics_t* get_statistics();

                /**
                 * @copydoc task_planner_t::succeeded()
                 */
                virtual bool succeeded() const;
                
                /**
                 * @copydoc task_planner_t::link_specification(specification_t*)
                 */
                virtual void link_specification(plan::specification_t* in_specification);

                /**
                 * @copydoc task_planner_t::link_query(query_t*)
                 */
                virtual void link_query(plan::query_t* in_query);

                /**
                 * @copydoc task_planner_t::resolve_query()
                 */
                virtual void resolve_query();

              protected:

                double calculate_heuristic(const util::space_point_t* state);

                /**
                 * @copydoc task_planner_t::update_vis_info()
                 */
                virtual void update_vis_info() const;

                util::space_point_t* heuristic_state;

                /** @brief The query for the task planner. */
                heuristic_task_specification_t* specification;
                /** @brief The query for the task planner. */
                plan::motion_planning_query_t* query;
                /** @brief An interruption criterion used for statistics gathering. */
                std::vector<plan::criterion_t*> stats_criteria;  

                plan::isst_t* real_motion_planner;
                plan::motion_planner_t* heuristic_planner;
                std::string real_planner_name, heuristic_planner_name;

                /** @brief The query for the heuristic planner */
                plan::motion_planning_query_t* heuristic_query;
                util::goal_state_t* heuristic_goal;


            };
        }

    }
}

#endif