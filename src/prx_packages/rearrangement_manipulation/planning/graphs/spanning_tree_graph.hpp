/**
 * @file astar_search.hpp 
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

#ifndef PRX_SPANNING_GRAPH_HPP
#define	PRX_SPANNING_GRAPH_HPP


#include <vector>
#include <set>

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * @Author: Athanasios Krontiris
             */
            class spanning_node_t : public util::undirected_node_t
            {

              public:

                spanning_node_t()
                {
                    explored = false;
                    pos_index = -1;
                }

                spanning_node_t(unsigned position_index, util::undirected_vertex_index_t v_pebble_node)
                {
                    pos_index = position_index;
                    v_pebble = v_pebble_node;
                    explored = false;
                }

                ~spanning_node_t(){ }

                void init_node(int position_index, util::undirected_vertex_index_t v_pebble_node)
                {
                    pos_index = position_index;
                    v_pebble = v_pebble_node;
                    explored = false;
                }

                unsigned pos_index;
                util::undirected_vertex_index_t v_pebble;
                bool explored;


            };

            class prx_spanning_tree_visitor_t : public boost::default_dfs_visitor
            {

              public:

                prx_spanning_tree_visitor_t()
                {
                    // PRX_INFO_S("create prx_spanning_tree_visitor");
                    tree_edges = new std::vector<util::undirected_edge_index_t > ();
                }

                ~prx_spanning_tree_visitor_t(){
                    // PRX_ERROR_S("DEstructor prx_spanning_tree_visitor");
                }

                void clear()
                {
                    // PRX_WARN_S("Delete prx_spanning_tree_visitor");
                    tree_edges->clear();
                    delete tree_edges;
                }

                template <class undirected_graph_t>
                void tree_edge(util::undirected_edge_index_t e, undirected_graph_t& g)
                {
                    // PRX_DEBUG_S("add e: " << e);
                    tree_edges->push_back(e);
                }

                std::vector<util::undirected_edge_index_t>* tree_edges;
            };

            struct prx_vertex_found_t
            {

                prx_vertex_found_t(util::undirected_vertex_index_t v, int pos_index = -1)
                {
                    v_found = v;
                    index = pos_index;
                }
                util::undirected_vertex_index_t v_found;
                int index;

            };

            class prx_search_visitor_t : public boost::default_bfs_visitor
            {

              public:

                prx_search_visitor_t(const util::undirected_graph_t* tree)
                {
                    inner_tree = tree;
                }

                ~prx_search_visitor_t(){
                    // PRX_ERROR_S("DEstructor prx_spanning_tree_visitor");
                }

                void clear(){
                    // PRX_WARN_S("Delete prx_spanning_tree_visitor");
                }

                void setup(bool looking_for_filled, const std::vector<unsigned>& current_positions)
                {
                    looking = looking_for_filled;
                    current = current_positions;
                }

                template <class Graph>
                void examine_vertex(util::undirected_vertex_index_t u, Graph& graph)
                {

                    unsigned index = inner_tree->get_vertex_as<spanning_node_t>(u)->pos_index;

                    bool has_object = false;
                    for( unsigned i = 0; i < current.size(); ++i )
                    {
                        if( looking && index == current[i] )
                            throw prx_vertex_found_t(u, i);

                        if( current[i] == index )
                        {
                            has_object = true;
                        }
                    }

                    if( !looking && !has_object )
                    {
                        throw prx_vertex_found_t(u);
                    }

                }

                                
                bool looking;
                std::vector<unsigned> current;
                const util::undirected_graph_t* inner_tree;

            };
        }
    }
}

#endif	


