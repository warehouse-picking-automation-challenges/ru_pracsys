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

#ifndef PRX_RRT_STAR_HPP
#define	PRX_RRT_STAR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/rrt/rrt_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"

namespace prx 
{ 
    namespace plan 
    {

class rrt_star_node_t : public rrt_node_t
{
};

class rrt_star_edge_t : public rrt_edge_t
{
};

    }
}

#endif
