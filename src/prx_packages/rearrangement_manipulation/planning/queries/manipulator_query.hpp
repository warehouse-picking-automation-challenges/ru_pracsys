/**
 * @file manipulator_query.hpp
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

#ifndef PRX_MANIPULATOR_QUERY_HPP
#define	PRX_MANIPULATOR_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "../../../manipulation/planning/queries/manipulation_query.hpp"

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
             * @anchor manipulator_query_t
             *
             * This class represents a problem instance to be solved by a manipulator task planner in the rearrangement problem.
             * It is a self-contained class which holds the starts/goals poses for the objects.
             * It stores all the plans and the constraints of the results.
             *
             * @brief <b> General query for manipulator task planners in the rearrangement problem. </b>
             *
             * @author Athanasios Krontiris
             */
            class manipulator_query_t : public manipulation::manipulation_query_t
            {

              public:

                manipulator_query_t();
                virtual ~manipulator_query_t();

                /** 
                 * @copydoc manipulation_query_t::clear() 
                 * 
                 * Its also clears the constraints that the query has stored.
                 */
                virtual void clear();

                /**
                 * Adds a constraint to a plan of the query.
                 * 
                 * @brief Adds a constraint to a plan of the query.
                 * 
                 * @param constraint Is the constraint that we want to add.
                 * @param index Is the position in the vector where the constraint will be added.
                 */
                virtual void add_constraint(unsigned constraint, unsigned index);

                /**
                 * Adds multiple constraints to a plan of the query.
                 * 
                 * @brief Adds multiple constraints to a plan of the query.
                 * 
                 * @param constraints Is the set of the constraints that we want to add.
                 * @param index Is the position in the vector where the constraint will be added.
                 */
                virtual void add_constraints(std::set<unsigned> new_constraints, unsigned index);

                std::vector<std::set<unsigned> > constraints;
                std::vector<std::set<unsigned> > full_constraints;
                std::vector<double> solutions_costs;
                unsigned from_pose;
                unsigned to_pose;
                
            };
        }
    }
}

#endif

