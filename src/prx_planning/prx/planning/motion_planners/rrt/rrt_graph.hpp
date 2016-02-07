/**
 * @file rrt_graph.hpp
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

#ifndef PRX_RRT_GRAPH_HPP
#define	PRX_RRT_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/tree.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/control.hpp"

namespace prx 
{ 
    namespace plan 
    {

/**
 * @brief <b> A node that adds necessary information to the tree.</b>
 * 
 * @author Zakary Littlefield
 */
class rrt_node_t : public util::tree_node_t
{

  public:
    rrt_node_t()
    {
        cost = 0;
    }
    ~rrt_node_t()
    {
        cost = 0;
    }
    
    /**
     * @brief Path cost from the start.
     */
    double cost;
};

/**
 * @brief <b> An edge that adds necessary information to the tree.</b>
 * 
 * @author Zakary Littlefield
 */
class rrt_edge_t : public util::tree_edge_t
{
  public:
    rrt_edge_t()
    {
    }

    ~rrt_edge_t(){ }
    
    /**
     * @brief The trajectory represented by this edge.
     */
    sim::trajectory_t trajectory;
    
    /**
     * @brief The plan that connects the two vertices.
     */
    sim::plan_t plan;
};


    }
}

#endif
