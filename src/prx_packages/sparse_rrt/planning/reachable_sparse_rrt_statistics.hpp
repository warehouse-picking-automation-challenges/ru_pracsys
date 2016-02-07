/**
 * @file reachable_sparse_rrt_statistics.hpp
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

#ifndef PRX_SPARSE_RRT_STATISTICS_HPP
#define	PRX_SPARSE_RRT_STATISTICS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/rrt/rrt_statistics.hpp"

namespace prx
{
    namespace packages
    {
        namespace sparse_rrt
        {
            /**
             * @brief A statistics class the provides information about the Sparse-RRT motion planner.
             * A statistics class the provides information about the Sparse-RRT motion planner.
             * 
             * @author Zakary Littlefield
             */
            class reachable_sparse_rrt_statistics_t : public plan::rrt_statistics_t
            {

              public:

                /** 
                 * @copydoc statistics_t::get_statistics() const 
                 */
                virtual std::string get_statistics() const;

                reachable_sparse_rrt_statistics_t();
                virtual ~reachable_sparse_rrt_statistics_t();

                /**
                 * @brief The average cost over all nodes.
                 */
                double average_cost;
            };

        }
    }
}

#endif
