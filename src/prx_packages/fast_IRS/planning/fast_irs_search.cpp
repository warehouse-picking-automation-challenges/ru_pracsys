/**
 * @file fast_irs_search.cpp 
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

#include "planning/fast_irs_search.hpp"
#include "prx/utilities/heuristic_search/astar_search.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include <algorithm>



namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace fast_irs
        {

            fast_irs_search_t::fast_irs_search_t()
            {
                tmp_astar_node = new astar_node_t();
            }

            fast_irs_search_t::fast_irs_search_t(undirected_graph_t *g)
            {
                link_graph(g);
                tmp_astar_node = new astar_node_t();
            }

            fast_irs_search_t::~fast_irs_search_t() { }

            void fast_irs_search_t::clear_structure()
            {
                undirected_vertex_index_t u;
                //foreach(u, all_marked )

                foreach(u, boost::vertices(graph->graph))
                {
                    initialize_vertex(u);
                    graph->distances[u] = PRX_INFINITY;
                    graph->colors[u] = WHITE;
                    graph->predecessors[u] = u;
                }
                //all_marked.clear();
                //blackened.clear();
                open_set.clear();

            }

            bool fast_irs_search_t::solve(undirected_vertex_index_t start, undirected_vertex_index_t goal)
            {
                bool returned = prm_astar_t::solve(start, goal);
                clear_structure();
                return returned;
            }

            bool fast_irs_search_t::multi_solve(const undirected_vertex_index_t start, const std::vector<undirected_vertex_index_t>& goals, double stretch)
            {
                undirected_vertex_index_t u, v;
                undirected_edge_index_t e;
                double dist;
                std::vector<undirected_vertex_index_t>::const_iterator goalIterator;

                //Need to set up the graph
                //foreach( u, blackened )

                foreach(u, boost::vertices(graph->graph))
                {
                    if( graph->colors[u] == BLACK )
                        graph->colors[u] = GRAY;
                }
                //blackened.clear();
                reached_goals.clear();
                //Also, need to initialize the goals
                std::vector< undirected_vertex_index_t > goal_set(goals.begin(), goals.end());

                double f = heuristic(start, goals);
                //PRX_DEBUG_COLOR("Begin multi-solve, inserting: " << (*graph)[start]->node_id << "  f: " << f, PRX_TEXT_GREEN );
                open_set.insert(new astar_node_t(start, f));
                graph->colors[start] = GRAY;
                graph->distances[start] = 0;
                discover_vertex(start);

                double reached_dist = 0;
                double length_limit = stretch * max_heuristic(start, goal_set);
                while( !open_set.empty() && reached_dist < length_limit && !goal_set.empty() )
                {
                    u = open_set.remove_min()->vertex;
                    examine_vertex(u);

                    //PRX_DEBUG_COLOR("Min node: " << (*graph)[u]->node_id << "  g: " << graph->distances[u] << "  h*stretch: " << length_limit , PRX_TEXT_GREEN );
                    std::vector< undirected_vertex_index_t >::iterator it = std::find(goal_set.begin(), goal_set.end(), u);

                    if( it != goal_set.end() )
                    {
                        //We should actually check to see if the spanner property is violated here before adding it as reached.
                        if( graph->distances[u] < stretch * heuristic(start, u) )
                        {
                            //Report this goal as found
                            reached_goals.push_back(u);
                        }
                        //Need to remove this goal from the set of goals
                        goal_set.erase(it);
                        //Then, we need to iterate over the entire open list and rebuild it
                        std::vector< astar_node_t *> open_set_elements;
                        open_set.get_items(open_set_elements);
                        //openSet.clear();

                        foreach(astar_node_t* n, open_set_elements)
                        {
                            //PRX_DEBUG_COLOR("Rebuilding: " << (*graph)[n.vertex]->node_id << "  f: " << n.f, PRX_TEXT_CYAN );
//                            tmp_astar_node->vertex = n->vertex;
//                            tmp_astar_node->f = graph->distances[n->vertex] + heuristic(n->vertex, goal_set);
                            open_set.update(n, new astar_node_t(n->vertex, graph->distances[n->vertex] + heuristic(n->vertex, goal_set)));
                        }
                    }

                    foreach(v, boost::adjacent_vertices(u, graph->graph))
                    {
                        e = boost::edge(u, v, graph->graph).first;
                        dist = graph->weights[e] + graph->distances[u];
                        if( examine_edge(u, e, v) && dist < graph->distances[v] )
                        {
                            graph->distances[v] = dist;
                            graph->predecessors[v] = u;
                            relaxed_edge(e);
                            if( graph->colors[v] == WHITE ) //We've never seen it before, add it to the list
                            {
                                //PRX_DEBUG_COLOR("White->Gray: " << (*graph)[v]->node_id << "  f: " << dist + heuristic(v, goal_set), PRX_TEXT_LIGHTGRAY );
                                graph->colors[v] = GRAY;
                                //all_marked.insert( v );
                                open_set.insert(new astar_node_t(v, dist + heuristic(v, goal_set)));
                                discover_vertex(v);
                            }
                            else if( graph->colors[v] == GRAY ) //It's in the open set or from a prior iteration, we might need to update it
                            {
//                                tmp_astar_node->vertex = v;
//                                tmp_astar_node->f = dist + heuristic(v, goal_set);
                                std::vector< astar_node_t *> open_set_elements;
                                open_set.get_items(open_set_elements);
                                //openSet.clear();
                                
                                unsigned found=0;
                                foreach(astar_node_t* n, open_set_elements)
                                {
                                    //PRX_DEBUG_COLOR("Rebuilding: " << (*graph)[n.vertex]->node_id << "  f: " << n.f, PRX_TEXT_CYAN );
                                    if(n->vertex==v)
                                    {
                                        found++;
                                        open_set.update(n, new astar_node_t(n->vertex,dist + heuristic(v, goal_set)));
                                    }
                                }
                                PRX_ASSERT(found<=1);
//                                if( open_set.contains(tmp_astar_node) )
//                                {
//                                    open_set.update(tmp_astar_node, tmp_astar_node);
//                                }
//                                else
                                if(found==0)
                                {
                                    //PRX_DEBUG_COLOR("Prior Iteration: " << (*graph)[v]->node_id << "  f: " << tmp_astar_node.f, PRX_TEXT_MAGENTA );
                                    open_set.insert(new astar_node_t(v,dist + heuristic(v, goal_set)));
                                }
                            }
                        }
                        else
                        {
                            not_relaxed_edge(e);
                        }
                    }

                    graph->colors[u] = BLACK;
                    finish_vertex(u);
                    //all_marked.insert( u );
                    //blackened.insert( u );
                    double nd = graph->distances[u];
                    if( reached_dist <= nd )
                        reached_dist = nd;
                }

                //PRX_DEBUG_COLOR("End multi_solve.", PRX_TEXT_GREEN );

                return false;
            }

            std::vector< undirected_vertex_index_t >& fast_irs_search_t::get_reached_goals()
            {
                return reached_goals;
            }

            double fast_irs_search_t::max_heuristic(util::undirected_vertex_index_t start, const std::vector< util::undirected_vertex_index_t > goals)
            {
                double maxH, h;
                size_t i;

                maxH = heuristic(start, goals[0]);

                for( i = 1; i < goals.size(); ++i )
                {
                    h = heuristic(start, goals[i]);
                    if( h > maxH )
                        maxH = h;
                }

                return maxH;
            }
        }
    }
}
