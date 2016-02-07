/**
 * @file rrt_statistics.hpp
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

#ifndef PRX_RRT_STATISTICS_HPP
#define	PRX_RRT_STATISTICS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/statistics/statistics.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @brief <b> A statistics class the provides information about the RRT motion planner.</b>
         * 
         * A statistics class the provides information about the RRT motion planner.
         * 
         * @author Zakary Littlefield
         */
        class rrt_statistics_t : public util::statistics_t
        {

          public:
            rrt_statistics_t();
            virtual ~rrt_statistics_t();

            /**
             *  @copydoc util::statistics_t::get_statistics() 
             */
            virtual std::string get_statistics() const;

            /**
             * @brief The number of vertices in the planner.
             */
            double num_vertices;

            /**
             * @brief The solution cost returned by the planner.
             */
            double solution_quality;
        };

    }
}

#endif
