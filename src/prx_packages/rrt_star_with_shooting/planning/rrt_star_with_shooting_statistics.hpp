/**
 * @file rrt_star_with_shooting_statistics.hpp
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

#ifndef PRX_RRT_STAR_SHOOTING_STATISTICS_HPP
#define	PRX_RRT_STAR_SHOOTING_STATISTICS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/rrt/rrt_statistics.hpp"

namespace prx
{
    namespace packages
    {
        namespace rrt_star_with_shooting
        {
            /**
             * @brief A statistics class the provides information about the RRT* using shooting.
             * A statistics class the provides information about the RRT* using shooting.
             * 
             * @author Zakary Littlefield
             */
            class rrt_star_with_shooting_statistics_t : public plan::rrt_statistics_t
            {

              public:
                rrt_star_with_shooting_statistics_t();
                virtual ~rrt_star_with_shooting_statistics_t();

                /**
                 * @copydoc statistics_t::get_statistics() const 
                 */
                virtual std::string get_statistics() const;

                /**
                 * @brief The average cost over all nodes.
                 */
                double average_cost;
            };

        }
    }
}

#endif
