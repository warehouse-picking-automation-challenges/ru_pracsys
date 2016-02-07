/**
 * @file rrt_star_with_shooting_graph.hpp
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

#ifndef PRX_RRT_STAR_SHOOTING_GRAPH_HPP
#define	PRX_RRT_STAR_SHOOTING_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/rrt/rrt_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"

namespace prx
{
    namespace packages
    {
        namespace rrt_star_with_shooting
        {

            class rrt_star_with_shooting_node_t : public plan::rrt_node_t
            {
                public:
                    rrt_star_with_shooting_node_t() : plan::rrt_node_t()
                    {
                        bridge = false;
                    }
                    /**
                     * @brief A flag that denotes if node is considered for nearest neighbor queries.
                     */
                    bool bridge;
            };

            class rrt_star_with_shooting_edge_t : public plan::rrt_edge_t
            {
            };

        }
    }
}

#endif
