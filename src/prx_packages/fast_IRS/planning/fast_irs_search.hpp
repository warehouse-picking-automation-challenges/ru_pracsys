/**
 * @file astar_search.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_FAST_IRS_SEARCH_HPP
#define PRX_FAST_IRS_SEARCH_HPP

#include "prx/utilities/heuristic_search/default_open_set.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/planning/motion_planners/prm_star/prm_astar.hpp"
#include <set>

namespace prx
{
    namespace packages
    {
        namespace fast_irs
        {
            /**
             * @anchor astar_search_t
             * 
             * Flexible implementation of the A* heuristic graph search algorithm. To use, 
             * derive from this class and provide an implementation of the single-goal 
             * heuristic function. The callback functions can also be overridden to give 
             * fine-grained information and control over the search process.
             * 
             * @brief <b> Flexible implementation of A* search. </b>
             * 
             * @author Andrew Dobson
             */
            class fast_irs_search_t : public plan::prm_astar_t
            {
                
            public:
                fast_irs_search_t();
                fast_irs_search_t( util::undirected_graph_t *g);
                virtual ~fast_irs_search_t();
                
                void clear_structure();

                /**
                 * Perform a query on the graph with multiple potential goals. The 
                 * first goal reached will be the one that terminates the search.
                 * 
                 * @brief Search the graph and terminate as soon as any of the goals are found.
                 * 
                 * @param start The start vertex.
                 * @param goals The list of goal vertices.
                 * @return True if a path was discovered, false if not.
                 */
                virtual bool multi_solve( util::undirected_vertex_index_t start, const std::vector< util::undirected_vertex_index_t>& goals, double stretch = PRX_INFINITY);
                
                virtual bool solve( util::undirected_vertex_index_t start, util::undirected_vertex_index_t goal );
                
                std::vector< util::undirected_vertex_index_t >& get_reached_goals();

                double max_heuristic( util::undirected_vertex_index_t start, const std::vector< util::undirected_vertex_index_t > goals );
                
                std::vector< util::undirected_vertex_index_t > reached_goals;

            protected:
                
                std::set< util::undirected_vertex_index_t > all_marked;
                std::set< util::undirected_vertex_index_t > blackened;
                
                util::astar_node_t* tmp_astar_node;
            };
        }
    }
}

#endif



