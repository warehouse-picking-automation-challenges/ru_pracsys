/**
 * @file pebble_solver.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_PEBBLE_SOLVER_HPP
#define	PRX_PEBBLE_SOLVER_HPP

#include "planning/graphs/pebble_graph.hpp"
#include "planning/modules/allocated_heap.hpp"

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/simulation/state.hpp"


#include <vector>
#include <set>

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            class pebble_solver_t
            {

              public:

                pebble_solver_t();
                void create_spanning_tree(const util::undirected_graph_t& source);
                bool is_on_goal(const std::vector<unsigned>& curr_config, const std::vector<unsigned>& goal_config);
                bool has_position(const std::vector<unsigned>& positions, unsigned query);
                void get_leaves();
                void BFS(util::undirected_vertex_index_t& found, util::undirected_vertex_index_t start, bool looking_for_filled);
                void resolve_query(std::vector< std::pair<util::undirected_vertex_index_t, util::undirected_vertex_index_t> >& plan_vertices, const util::undirected_graph_t& pebble_graph, std::vector<unsigned>& start_config, std::vector<unsigned> goal_config, const util::space_t* object_space);

              protected:
                util::undirected_graph_t spanning_tree;
                std::vector<unsigned> g_config;
                std::vector<unsigned> current_config;
                std::vector< std::pair<int, int> > pebble_plan;
                allocated_heap_t<util::undirected_vertex_index_t> leaves;
//                std::vector< std::pair<util::undirected_vertex_index_t, util::undirected_vertex_index_t> > auletta_path;


            };
        }
    }
}



#endif	/* PEBBLE_SOLVER_HPP */

