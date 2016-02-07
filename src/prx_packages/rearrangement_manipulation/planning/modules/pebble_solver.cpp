/**
 * @file undirected_node.hpp
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

#include "planning/modules/pebble_solver.hpp"
#include "planning/graphs/spanning_tree_graph.hpp"

#include "prx/utilities/definitions/random.hpp"

#include <boost/graph/connected_components.hpp>
#include <boost/graph/depth_first_search.hpp>
#include<queue>
#include <boost/graph/compressed_sparse_row_graph.hpp>
#include <boost/graph/subgraph.hpp>

namespace prx
{
    //    using namespace std;
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {

        namespace rearrangement_manipulation
        {

            pebble_solver_t::pebble_solver_t() { }

            void pebble_solver_t::create_spanning_tree(const undirected_graph_t& p_graph)
            {
                hash_t<undirected_vertex_index_t, undirected_vertex_index_t> same_nodes;

                foreach(undirected_vertex_index_t v, boost::vertices(p_graph.graph))
                {
                    undirected_vertex_index_t v_span = spanning_tree.add_vertex<spanning_node_t > ();
                    spanning_tree.get_vertex_as<spanning_node_t > (v_span)->init_node(p_graph.get_vertex_as<pebble_node_t > (v)->position_index, v);
                    same_nodes[v] = v_span;

                }

                prx_spanning_tree_visitor_t* tree_visitor = new prx_spanning_tree_visitor_t();
                depth_first_search<
                        undirected_graph_type,
                        undirected_graph_t,
                        undirected_vertex_index_t,
                        undirected_edge_index_t,
                        prx_spanning_tree_visitor_t
                        > (&p_graph, tree_visitor);

                foreach(undirected_edge_index_t e, *(tree_visitor->tree_edges))
                {
                    PRX_DEBUG_COLOR("edge : " << p_graph.get_vertex_as<pebble_node_t > (boost::source(e, p_graph.graph))->position_index << " - " << p_graph.get_vertex_as<pebble_node_t > (boost::target(e, p_graph.graph))->position_index << "    degree : " << boost::out_degree(boost::target(e, p_graph.graph), p_graph.graph), PRX_TEXT_BLUE);
                    spanning_tree.add_edge<undirected_edge_t > (same_nodes[boost::source(e, p_graph.graph)], same_nodes[boost::target(e, p_graph.graph)], 1);
                }
            }

            bool pebble_solver_t::is_on_goal(const std::vector<unsigned>& curr_config, const std::vector<unsigned>& goal_config)
            {
                unsigned counter = 0;
                for( unsigned i = 0; i < curr_config.size(); ++i )
                {
                    for( unsigned j = 0; j < goal_config.size(); j++ )
                    {
                        if( curr_config[i] == goal_config[j] )
                        {
                            counter++;
                        }
                    }
                }
                return counter == goal_config.size();
            }

            bool pebble_solver_t::has_position(const std::vector<unsigned>& positions, unsigned query)
            {
                for( unsigned i = 0; i < positions.size(); ++i )
                {
                    if( query == positions[i] )
                    {
                        return true;
                    }
                }
                return false;
            }

            void pebble_solver_t::get_leaves()
            {

                foreach(undirected_vertex_index_t v, boost::vertices(spanning_tree.graph))
                {
                    PRX_DEBUG_COLOR("Vertex : " << spanning_tree.get_vertex_as<spanning_node_t > (v)->pos_index << "   has degree : " << boost::degree(v, spanning_tree.graph), PRX_TEXT_GREEN);
                    if( boost::degree(v, spanning_tree.graph) <= 1 )
                        leaves.push_back(v);

                }
            }

            void pebble_solver_t::BFS(undirected_vertex_index_t& found, undirected_vertex_index_t start, bool looking_for_filled)
            {

                foreach(undirected_vertex_index_t u, boost::vertices(spanning_tree.graph))
                {
                    spanning_tree.predecessors[u] = u;
                    spanning_tree.get_vertex_as<spanning_node_t > (u)->explored = false;
                }

                spanning_tree.get_vertex_as<spanning_node_t > (start)->explored = true;

                std::queue<undirected_vertex_index_t> Q;

                Q.push(start);

                while( Q.size() != 0 )
                {
                    undirected_vertex_index_t u = Q.front();
                    Q.pop();

                    foreach(undirected_vertex_index_t v, boost::adjacent_vertices(u, spanning_tree.graph))
                    {
                        spanning_node_t* current = spanning_tree.get_vertex_as<spanning_node_t > (v);
                        if( current->explored == false )
                        {
                            spanning_tree.predecessors[v] = u;
                            Q.push(v);
                            current->explored = true;
                            bool filled = has_position(current_config, current->pos_index);
                            if( (looking_for_filled && filled) || (!looking_for_filled && !filled) )
                            {
                                found = v;
                                return;
                            }
                        }
                    }
                }

            }

            std::string print_config(std::vector<unsigned> config)
            {
                std::stringstream output(std::stringstream::out);
                for( unsigned i = 0; i < config.size() - 1; ++i )
                {
                    output << config[i] << " , ";
                }
                output << config.back();
                return output.str();
            }

            void pebble_solver_t::resolve_query(std::vector< std::pair<util::undirected_vertex_index_t, util::undirected_vertex_index_t> >& plan_vertices, const undirected_graph_t& pebble_graph, std::vector<unsigned>& start_config, std::vector<unsigned> goal_config, const util::space_t* object_space)
            {
                plan_vertices.clear();
                //                undirected_graph_t pebble_graph;
                //                undirected_vertex_index_t v1 = pebble_graph.add_vertex<pebble_node_t>();
                //                pebble_graph.get_vertex_as<pebble_node_t>(v1)->position_index = 1;
                //                undirected_vertex_index_t v2 = pebble_graph.add_vertex<pebble_node_t>();
                //                pebble_graph.get_vertex_as<pebble_node_t>(v2)->position_index = 2;
                //                undirected_vertex_index_t v3 = pebble_graph.add_vertex<pebble_node_t>();
                //                pebble_graph.get_vertex_as<pebble_node_t>(v3)->position_index = 3;
                //                undirected_vertex_index_t v4 = pebble_graph.add_vertex<pebble_node_t>();
                //                pebble_graph.get_vertex_as<pebble_node_t>(v4)->position_index = 4;
                //                undirected_vertex_index_t v5 = pebble_graph.add_vertex<pebble_node_t>();
                //                pebble_graph.get_vertex_as<pebble_node_t>(v5)->position_index = 5;
                //                undirected_vertex_index_t v6 = pebble_graph.add_vertex<pebble_node_t>();
                //                pebble_graph.get_vertex_as<pebble_node_t>(v6)->position_index = 6;
                //                undirected_vertex_index_t v7 = pebble_graph.add_vertex<pebble_node_t>();
                //                pebble_graph.get_vertex_as<pebble_node_t>(v7)->position_index = 7;
                //                undirected_vertex_index_t v8 = pebble_graph.add_vertex<pebble_node_t>();
                //                pebble_graph.get_vertex_as<pebble_node_t>(v8)->position_index = 8;
                //                undirected_vertex_index_t v9 = pebble_graph.add_vertex<pebble_node_t>();
                //                pebble_graph.get_vertex_as<pebble_node_t>(v9)->position_index = 9;
                //
                //                pebble_graph.add_edge<pebble_edge_t>(v1, v2, 1);
                //                pebble_graph.add_edge<pebble_edge_t>(v2, v1, 1);
                //
                //                pebble_graph.add_edge<pebble_edge_t>(v1, v3, 1);
                //                pebble_graph.add_edge<pebble_edge_t>(v3, v1, 1);
                //
                //                pebble_graph.add_edge<pebble_edge_t>(v1, v4, 1);
                //                pebble_graph.add_edge<pebble_edge_t>(v4, v1, 1);
                //
                //                pebble_graph.add_edge<pebble_edge_t>(v2, v3, 1);
                //                pebble_graph.add_edge<pebble_edge_t>(v3, v2, 1);
                //
                //                pebble_graph.add_edge<pebble_edge_t>(v2, v5, 1);
                //                pebble_graph.add_edge<pebble_edge_t>(v5, v2, 1);
                //
                //                pebble_graph.add_edge<pebble_edge_t>(v2, v6, 1);
                //                pebble_graph.add_edge<pebble_edge_t>(v6, v2, 1);
                //
                //
                //                pebble_graph.add_edge<pebble_edge_t>(v4, v6, 1);
                //                pebble_graph.add_edge<pebble_edge_t>(v6, v4, 1);
                //
                //                pebble_graph.add_edge<pebble_edge_t>(v4, v5, 1);
                //                pebble_graph.add_edge<pebble_edge_t>(v5, v4, 1);
                //
                //                pebble_graph.add_edge<pebble_edge_t>(v7, v8, 1);
                //                pebble_graph.add_edge<pebble_edge_t>(v8, v7, 1);

                if( is_on_goal(start_config, goal_config) )
                    return;

                for( unsigned i = 0; i < start_config.size(); ++i )
                    current_config.push_back(start_config[i]);

                create_spanning_tree(pebble_graph);
                leaves.resize(boost::num_vertices(spanning_tree.graph));
                prx_search_visitor_t* search_visitor = new prx_search_visitor_t(&spanning_tree);
                undirected_vertex_index_t v;
                leaves.resize(boost::num_vertices(spanning_tree.graph) + 1);
                get_leaves();
                PRX_DEBUG_COLOR("leaves size : " << leaves.print(), PRX_TEXT_BROWN);
                PRX_DEBUG_COLOR("----------------------  [" << print_config(current_config) << "] -> [" << print_config(goal_config) << "]", PRX_TEXT_RED);
                do
                {
                    v = leaves.pop_front();

                    spanning_node_t* node = spanning_tree.get_vertex_as<spanning_node_t > (v);
                    PRX_DEBUG_COLOR("#######################################  " << print_config(current_config), PRX_TEXT_MAGENTA);
                    PRX_DEBUG_COLOR("v pop : " << node->pos_index, PRX_TEXT_BROWN);
                    PRX_DEBUG_COLOR("leaves_size: " << leaves.size(), PRX_TEXT_BROWN);
                    if( !has_position(goal_config, node->pos_index) && has_position(current_config, node->pos_index) )
                    {

                        PRX_DEBUG_COLOR("looking for empty spot", PRX_TEXT_CYAN);
                        search_visitor->setup(false, current_config);

                        undirected_vertex_index_t v_prev;

                        undirected_vertex_index_t v_f;
                        BFS(v_f, v, false);

                        foreach(undirected_vertex_index_t v_temp, boost::vertices(spanning_tree.graph))
                        {
                            PRX_DEBUG_COLOR("##### PRED INFO : Vertex : " << spanning_tree.get_vertex_as<spanning_node_t > (v_temp)->pos_index << "   has predecessor : " << spanning_tree.get_vertex_as<spanning_node_t > (spanning_tree.predecessors[v_temp])->pos_index, PRX_TEXT_GREEN);

                        }
                        for( undirected_vertex_index_t v_current = v_f; v_current != v; v_current = spanning_tree.predecessors[v_current] )
                        {
                            v_prev = spanning_tree.predecessors[v_current];
                            PRX_ASSERT(v_prev != v_current);
                            PRX_DEBUG_COLOR("v_prev : " << spanning_tree.get_vertex_as<spanning_node_t > (v_prev)->pos_index << "    v_current: " << spanning_tree.get_vertex_as < spanning_node_t > (v_current)->pos_index, PRX_TEXT_BROWN);
                            undirected_edge_index_t e = boost::edge(spanning_tree.get_vertex_as<spanning_node_t > (v_prev)->v_pebble, spanning_tree.get_vertex_as<spanning_node_t > (v_current)->v_pebble, pebble_graph.graph).first;
                            plan_vertices.push_back(std::make_pair(spanning_tree.get_vertex_as<spanning_node_t > (v_prev)->v_pebble, spanning_tree.get_vertex_as<spanning_node_t > (v_current)->v_pebble));

                            while( plan_vertices.size() > 1 && plan_vertices.back().first == plan_vertices[plan_vertices.size() - 2].second && boost::edge(plan_vertices[plan_vertices.size() - 2].first, plan_vertices.back().second, pebble_graph.graph).second )
                            {
                                PRX_DEBUG_COLOR("Insert <" << pebble_graph.get_vertex_as<pebble_node_t > (plan_vertices[plan_vertices.size() - 2].first)->position_index << "," << pebble_graph.get_vertex_as<pebble_node_t > (plan_vertices.back().second)->position_index << "> and deleting <" << pebble_graph.get_vertex_as<pebble_node_t > (plan_vertices[plan_vertices.size() - 2].first)->position_index << "," << pebble_graph.get_vertex_as<pebble_node_t > (plan_vertices[plan_vertices.size() - 2].second)->position_index << "> and <" << pebble_graph.get_vertex_as<pebble_node_t > (plan_vertices[plan_vertices.size() - 1].first)->position_index << "," << pebble_graph.get_vertex_as<pebble_node_t > (plan_vertices[plan_vertices.size() - 1].second)->position_index << ">", PRX_TEXT_CYAN);
                                std::pair<util::undirected_vertex_index_t, util::undirected_vertex_index_t> temp_pair(plan_vertices[plan_vertices.size() - 2].first, plan_vertices.back().second);
                                plan_vertices.pop_back();
                                plan_vertices.pop_back();
                                plan_vertices.push_back(temp_pair);
                            }
                        }

                        PRX_ASSERT(v != v_f);
                        unsigned temp_pos_index = spanning_tree.get_vertex_as<spanning_node_t > (v)->pos_index;
                        PRX_DEBUG_COLOR("SHIFT TO " << temp_pos_index, PRX_TEXT_RED);
                        for( unsigned i = 0; i < current_config.size(); ++i )
                        {
                            if( current_config[i] == temp_pos_index )
                            {
                                PRX_DEBUG_COLOR(i << ") current before : " << current_config[i] << " -> " << spanning_tree.get_vertex_as<spanning_node_t > (v_f)->pos_index, PRX_TEXT_RED);
                                current_config[i] = spanning_tree.get_vertex_as<spanning_node_t > (v_f)->pos_index;
                                PRX_DEBUG_COLOR("Curr config : " << print_config(current_config), PRX_TEXT_BLUE);
                                break;
                            }
                        }
                    }
                    else if( has_position(goal_config, node->pos_index) && !has_position(current_config, node->pos_index) )
                    {
                        PRX_DEBUG_COLOR("looking for filled spot", PRX_TEXT_CYAN);
                        search_visitor->setup(false, current_config);

                        undirected_vertex_index_t v_prev;

                        undirected_vertex_index_t v_f;
                        BFS(v_f, v, true);

                        foreach(undirected_vertex_index_t v_temp, boost::vertices(spanning_tree.graph))
                        {
                            PRX_DEBUG_COLOR("##### PRED INFO : Vertex : " << spanning_tree.get_vertex_as<spanning_node_t > (v_temp)->pos_index << "   has predecessor : " << spanning_tree.get_vertex_as<spanning_node_t > (spanning_tree.predecessors[v_temp])->pos_index, PRX_TEXT_GREEN);

                        }
                        for( undirected_vertex_index_t v_current = v_f; v_current != v; v_current = spanning_tree.predecessors[v_current] )
                        {
                            v_prev = spanning_tree.predecessors[v_current];
                            PRX_ASSERT(v_prev != v_current);
                            PRX_DEBUG_COLOR("v_current : " << spanning_tree.get_vertex_as<spanning_node_t > (v_current)->pos_index << "    v_prev : " << spanning_tree.get_vertex_as < spanning_node_t > (v_prev)->pos_index, PRX_TEXT_BROWN);
                            undirected_edge_index_t e = boost::edge(spanning_tree.get_vertex_as<spanning_node_t > (v_current)->v_pebble, spanning_tree.get_vertex_as<spanning_node_t > (v_prev)->v_pebble, pebble_graph.graph).first;
                            plan_vertices.push_back(std::make_pair(spanning_tree.get_vertex_as<spanning_node_t > (v_current)->v_pebble, spanning_tree.get_vertex_as<spanning_node_t > (v_prev)->v_pebble));

                            while( plan_vertices.size() > 1 && plan_vertices.back().first == plan_vertices[plan_vertices.size() - 2].second && boost::edge(plan_vertices[plan_vertices.size() - 2].first, plan_vertices.back().second, pebble_graph.graph).second )
                            {
                                //                                PRX_DEBUG_COLOR("Insert <"<<pebble_graph.get_vertex_as<pebble_node_t>(plan_vertices[plan_vertices.size() - 2].first)->position_index<<","<<pebble_graph.get_vertex_as<pebble_node_t>(plan_vertices.back().second)->position_index<<"> and deleting <"<<pebble_graph.get_vertex_as<pebble_node_t>(plan_vertices[plan_vertices.size() - 2].first)->position_index<<","<<pebble_graph.get_vertex_as<pebble_node_t>(plan_vertices[plan_vertices.size()-2].second)->position_index<<"> and <"<<pebble_graph.get_vertex_as<pebble_node_t>(plan_vertices[plan_vertices.size() - 1].first)->position_index<<","<<pebble_graph.get_vertex_as<pebble_node_t>(plan_vertices[plan_vertices.size()-1].second)->position_index<<">",PRX_TEXT_CYAN);
                                std::pair<util::undirected_vertex_index_t, util::undirected_vertex_index_t> temp_pair(plan_vertices[plan_vertices.size() - 2].first, plan_vertices.back().second);
                                plan_vertices.pop_back();
                                plan_vertices.pop_back();
                                plan_vertices.push_back(temp_pair);


                            }
                            //                            PRX_DEBUG_COLOR("Computed Path (" << computed_path.size() << ")" << ", Edge (" << pebble_graph.get_edge_as<pebble_edge_t > (e)->path.size() << ")", PRX_TEXT_GREEN);
                        }

                        PRX_ASSERT(v != v_f);
                        unsigned temp_pos_index = spanning_tree.get_vertex_as<spanning_node_t > (v_f)->pos_index;
                        PRX_DEBUG_COLOR("SHIFT TO " << temp_pos_index, PRX_TEXT_RED);
                        for( unsigned i = 0; i < current_config.size(); ++i )
                        {
                            if( current_config[i] == temp_pos_index )
                            {
                                PRX_DEBUG_COLOR("Before: " << print_config(current_config), PRX_TEXT_GREEN);
                                unsigned keep_debug =  current_config[i];
                                current_config[i] = spanning_tree.get_vertex_as<spanning_node_t > (v)->pos_index;
                                PRX_DEBUG_COLOR(i << ") current before : " << keep_debug << " -> " << spanning_tree.get_vertex_as<spanning_node_t > (v)->pos_index, PRX_TEXT_RED);
                                PRX_DEBUG_COLOR("After : " << print_config(current_config), PRX_TEXT_CYAN);
                                break;
                            }
                        }
                    }                   
                    std::vector<undirected_vertex_index_t> adj_vertices;
                    foreach(undirected_vertex_index_t v_leaf_adj, boost::adjacent_vertices(v, spanning_tree.graph))
                    {
                        adj_vertices.push_back(v_leaf_adj);
                    }
                    
                    spanning_tree.clear_and_remove_vertex(v);
                    foreach(undirected_vertex_index_t v_leaf_adj, adj_vertices)
                    {
                        PRX_DEBUG_COLOR("ADJ: " << spanning_tree.get_vertex_as<spanning_node_t > (v_leaf_adj)->pos_index << "    v:" << v_leaf_adj << "    degree:" << boost::degree(v_leaf_adj, spanning_tree.graph), PRX_TEXT_CYAN);
                        if( boost::degree(v_leaf_adj, spanning_tree.graph) == 1 )
                        {
                            leaves.push_back(v_leaf_adj);
                        }
                    }
                    
                    PRX_DEBUG_COLOR(" End leaves_size: " << leaves.print(), PRX_TEXT_LIGHTGRAY);
                    
                }
                while( !is_on_goal(current_config, goal_config) );

                //                trajectory_t tmp_traj=computed_path;
                //                trajectory_t tmp_traj2=computed_path;
                //                
                //                tmp_traj2.chop(computed_path.size()/10);
                //                tmp_traj.splice(tmp_traj.begin()+2,tmp_traj.end()-2,)
                //                

                PRX_DEBUG_COLOR("COMPLETE AULETTA'S PATH : " << plan_vertices.size(), PRX_TEXT_RED);                
            }
        }
    }
}
