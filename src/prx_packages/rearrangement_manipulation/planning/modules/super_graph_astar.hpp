/**
 * @file super_graph_astar.hpp
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
#pragma once

#ifndef PRX_SUPER_GRAPH_ASTAR_HPP
#define	PRX_SUPER_GRAPH_ASTAR_HPP


#include "prx/planning/modules/heuristic_search/astar_module.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/heuristic_search/astar_open_set.hpp"

#include <set>
#include <boost/range/adaptor/map.hpp>


/**
 * Writing this assuming the following variables are (somehow) available:
 *
 * directed_graph_t graph (With node and edge types from arrange_graph.hpp)
 * system_name_validity_checker_t naming_validity_checker
 * std::vector< directed_vertex_index_t > OA_predecessor_map
 */

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            class super_graph_astar_t : public plan::astar_module_t
            {

              public:
                super_graph_astar_t();
                
                virtual double heuristic(util::undirected_vertex_index_t current, util::undirected_vertex_index_t goal);
            };
        }
    }
}

#endif
