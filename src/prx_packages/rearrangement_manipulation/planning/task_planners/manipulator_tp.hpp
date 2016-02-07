/**
 * @file manipulator_tp_t.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_MANIPULATOR_TP_HPP
#define	PRX_MANIPULATOR_TP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"

#include "../../../manipulation/planning/task_planners/manipulation_tp.hpp"

namespace prx
{

    namespace util
    {
        class bounds_t;
        class goal_state_t;
        class statistics_t;
    }

    namespace plan
    {
        class motion_planning_specification_t;
        class motion_planning_query_t;
    }

    namespace packages
    {
        namespace manipulation
        {
            class manip_sampler_t;
        }
        
        namespace rearrangement_manipulation
        {
            using namespace baxter;
            class manipulator_specification_t;
            class manipulator_query_t;
            class manipulator_mp_query_t;

            /**
             * Manipulator task planner. Computes the path for moving an object from an 
             * initial to a target position.             
             * 
             * @autors Athanasios Krontiris
             */
            class manipulator_tp_t : public manipulation::manipulation_tp_t
            {

              public:

                manipulator_tp_t();
                virtual ~manipulator_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc motion_planner_t::reset() 
                 */
                virtual void reset();

                /**
                 * @copydoc motion_planner_t::get_statistics()
                 */
                virtual const util::statistics_t* get_statistics();

                /**
                 * @copydoc planner_t::link_specification(specification_t*)
                 */
                virtual void link_specification(plan::specification_t* new_spec);

                /**
                 * @copydoc motion_planner_t::link_query()
                 */
                virtual void link_query(plan::query_t* new_query);

                /** 
                 * @copydoc motion_planner_t::setup() 
                 * 
                 * Will occupy memory for the random_open_point and the new_control, after 
                 * planning_query has been linked. 
                 */
                virtual void setup();

                /**
                 * @copydoc motion_planner_t::execute()
                 */
                virtual bool execute();
                
                bool serialize();
                bool deserialize();

              protected:
                /** @brief The specification for this manipulation problem */
                manipulator_specification_t* in_specs;

                /** @brief The query for this manipulation problem */
                manipulator_query_t* in_query;
                
                /** @brief Helping variable for the motion planners' queries.*/
                manipulator_mp_query_t* manip_mp_query;
                
                std::set<unsigned>* transit_constraints;                

                virtual bool retract(manipulation::pose_t& pose, plan::query_t* query, int grasp_index = 0);
                virtual void update_query_information(plan::query_t* query, int index = -1);
                virtual void combine_queries(unsigned& min_path_size);
                virtual void add_null_path();
            };
        }
    }
}


#endif	
