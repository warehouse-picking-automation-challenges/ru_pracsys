/**
 * @file manipulator_mp_query.hpp
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
#pragma once

#ifndef PRX_MANIPULATOR_MOTION_PLANNER_QUERY_HPP
#define	PRX_MANIPULATOR_MOTION_PLANNER_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/modules/heuristic_search/astar_module.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace plan
    {
        class stopping_criteria_t;
    }

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * @anchor manipulator_mp_query_t
             *            
             *
             * @author Athanasios Krontiris
             */
            class manipulator_mp_query_t : public plan::motion_planning_query_t
            {

              public:

                manipulator_mp_query_t();
                virtual ~manipulator_mp_query_t();

                /** 
                 * @copydoc motion_planning_query_t::void clear() 
                 * 
                 * Its also clears the constraints that the query has stored.
                 */
                virtual void clear();

                /**
                 * Adds a constraint to the query.
                 * 
                 * @brief Adds a constraint to the query.
                 * 
                 * @param constraint Is the constraint that we want to add.
                 */
                virtual void add_constraint(unsigned constraint);

                /**
                 * Adds multiple constraints to the query.
                 * 
                 * @brief Adds multiple constraints to the query.
                 * 
                 * @param constraints Is the set of the constraints that we want to add.
                 */
                virtual void add_constraints(std::set<unsigned> new_constraints);
                
                /**
                 * This function sets an extra constraint. 
                 * This function mostly used when we request full manipulation path from the manipulator task planner.
                 * 
                 * @params constraint The extra constraint
                 */
                virtual void set_extra_constraint(unsigned constraint);
                
                plan::astar_module_t::astar_mode search_mode;
                std::set<unsigned> constraints;
                std::set<unsigned> full_constraints;      
                bool has_extra_constraint;
                unsigned extra_constraint;
            };
        }
    }
}

#endif

