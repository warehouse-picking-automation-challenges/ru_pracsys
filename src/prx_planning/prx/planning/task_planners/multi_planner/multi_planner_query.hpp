/**
 * @file multi_planner_query.hpp
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

#ifndef PRX_MULTI_PLANNER_QUERY_HPP
#define PRX_MULTI_PLANNER_QUERY_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @anchor multi_planner_query_t
         *
         * A query specificially for the \ref multi_planner_t task planner.  It stores
         * a list of planner queries to be answered sequentially by different planners.
         *
         * @brief <b> Queries to be used with the multi-planner task planner. </b>
         *
         * @author Zakary Littlefield
         */
        class multi_planner_query_t : public query_t
        {

          public:
            /**
             * @copydoc query_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

            /** @brief A list of motion planning queries to be answered. */
            util::hash_t<std::string, query_t* > planner_queries;
        };

    }
}

#endif 