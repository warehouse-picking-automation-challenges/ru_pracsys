/**
 * @file super_graph_astar.cpp
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

#include "planning/modules/super_graph_astar.hpp"
#include "planning/modules/system_name_validity_checker.hpp"
#include "planning/graphs/manipulation_graph.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"
#include "planning/graphs/pebble_graph.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::super_graph_astar_t, prx::plan::astar_module_t)

namespace prx
{
    namespace packages
    {
        using namespace util;
        using namespace plan;

        namespace rearrangement_manipulation
        {

            super_graph_astar_t::super_graph_astar_t()
            {

            }        

            double super_graph_astar_t::heuristic(util::undirected_vertex_index_t current, util::undirected_vertex_index_t goal)
            {
                return 0;
            }
        }
    }
}
