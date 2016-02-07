/**
 * @file boost_wrappers.hpp
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

#ifndef PRX_BOOST_WRAPPERS_HPP
#define	PRX_BOOST_WRAPPERS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/utilities/graph/directed_graph.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"

#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/depth_first_search.hpp>

namespace prx
{
    namespace util
    {

        struct prx_found_goal_t
        {

        };

        /**
         * A default distance heuristic for use in Boost graph searches. Works
         * with the graph classes.
         * @brief <b> A distance heuristic for Boost graph searches. </b>
         * @author Athanasios Krontiris
         */
        template<class Graph, class PRX_Graph, class Vertex, class Node>
        class distance_heuristic_t : public boost::astar_heuristic<Graph, double>
        {

          public:

            /**
             * @brief Constructor
             * @param g The graph used for determining distances.
             * @param goal The goal vertex.
             * @param metric A distance metric used for distance calculations.
             */
            distance_heuristic_t(const PRX_Graph * g, Vertex goal, distance_metric_t* metric)
            {
                inner_graph = g;
                goal_node = g->operator[](goal);
                distance_metric = metric;
            }

            /**
             * Sets the goal for this heuristic.
             * @brief Sets the goal for this heuristic.
             * @param goal
             */
            void set_goal(Vertex goal)
            {
                goal_node = inner_graph->operator[](goal);
            }

            /**
             * Performs a heuristic calculation from a given vertex.
             * 
             * @brief Performs a heuristic calculation from a given vertex.
             * @param u The vertex to estimate distance to the goal.
             * @return The estimated distance to the goal.
             */
            double operator()(Vertex u)
            {
                if( distance_metric == NULL )
                    return 0;
                return distance_metric->distance_function(inner_graph->operator[](u)->point, goal_node->point);
            }

          private:
            /**
             * @brief The goal node for this heuristic. 
             */
            Node* goal_node;
            /**
             * @brief The distance metric to calculate distances.
             */
            distance_metric_t* distance_metric;
            /**
             * @brief The graph that contains the vertices for the search.
             */
            const PRX_Graph * inner_graph;
        };

        /**
         * A default visitor for A* searches in Boost.
         * @brief <b> A default visitor for A* searches in Boost. </b>
         */
        template<class Graph, class Vertex>
        class prx_astar_goal_visitor : public boost::default_astar_visitor
        {

          public:

            prx_astar_goal_visitor(){ }

            /**
             * @brief Constructor
             * @param goal The goal vertex to find in the graph.
             */
            prx_astar_goal_visitor(Vertex goal) : m_goal(goal){ }

            /**
             * Sets the goal for this visitor.
             * @brief Sets the goal for this visitor.
             * @param goal
             */
            void set_goal(Vertex goal)
            {
                m_goal = goal;
            }

            /**
             * The function called by Boost graph searches when a node is expanded.
             * @param u The vertex being expanded.
             * @param g The graph being searched.
             */
            template <class Graph_t>
            void examine_vertex(Vertex u, Graph_t& g)
            {
                if( u == m_goal )
                    throw prx_found_goal_t();
            }

          private:
            /**
             * The goal vertex the search procedure is looking for.
             */
            Vertex m_goal;
        };

        /**
         * A wrapper around the Boost A* search. 
         * @brief A wrapper around the Boost A* search. 
         * 
         * @param g The graph to search.
         * @param x_start The start vertex.
         * @param x_goal The goal vertex.
         * @param metric A distance metric for computing distances between vertices.
         */
        template<class Graph, class PRX_Graph, class Vertex, class Node>
        void astar_search(PRX_Graph& g, Vertex x_start, Vertex x_goal, distance_metric_t* metric = NULL)
        {

            foreach(Vertex v, boost::vertices(g.graph))
            {
                g.colors[v] = boost::color_traits<boost::default_color_type > ().white();
                g.predecessors[v] = v;
            }
            boost::astar_search(g.graph, x_start,
                                distance_heuristic_t<Graph, PRX_Graph, Vertex, Node > (&g, x_goal, metric),
                                boost::predecessor_map(g.predecessors).
                                distance_map(g.distances).
                                vertex_index_map(g.indices).
                                color_map(g.colors).
                                visitor(prx_astar_goal_visitor<Graph, Vertex > (x_goal)));
        }

        /**
         * A wrapper around the Boost A* search. 
         * @param g A pointer to a graph to search.
         * @param x_start The start vertex.
         * @param x_goal The goal vertex.
         * @param metric A distance metric for computing distances between vertices.
         */
        template<class Graph, class PRX_Graph, class Vertex, class Node>
        void astar_search(const PRX_Graph* g, Vertex x_start, Vertex x_goal, distance_metric_t* metric = NULL)
        {

            foreach(Vertex v, boost::vertices(g->graph))
            {
                g->colors[v] = boost::color_traits<boost::default_color_type > ().white();
                g->predecessors[v] = v;
            }
            boost::astar_search(g->graph, x_start,
                                distance_heuristic_t<Graph, PRX_Graph, Vertex, Node > (g, x_goal, metric),
                                boost::predecessor_map(g->predecessors).
                                distance_map(g->distances).
                                vertex_index_map(g->indices).
                                color_map(g->colors).
                                visitor(prx_astar_goal_visitor<Graph, Vertex > (x_goal)));
        }

        //We don't know why boost needs the first template class but IT NEEDS IT. DO NOT REMOVE.
        // Graph : graph_type (ex. directed_graph_type)
        // PRX_Graph : graph_t (ex. directed_graph_t)
        // Vertex : the vertex index type (ex. directed_vertex_index_t)
        // Heuristic: the heuristic type that you want to use
        // Visitor : Visitor's type that you will use

        /**
         * A wrapper around the Boost A* search.
         * @param g A graph to search.
         * @param start The start vertex.
         * @param heuristic A pre-constructed heuristic.
         * @param astar_visitor A pre-constructed visitor.
         */
        template<class Graph, class PRX_Graph, class Vertex, class Heuristic, class Visitor>
        void astar_search(PRX_Graph& g, Vertex start, Heuristic* heuristic, Visitor* astar_visitor)
        {

            foreach(Vertex v, boost::vertices(g.graph))
            {
                g.colors[v] = boost::color_traits<boost::default_color_type > ().white();
                g.predecessors[v] = v;
            }
            //    PRX_INFO_S("Performing an A* search with new heuristic and new visitor ... ");    
            boost::astar_search(g.graph, start,
                                *heuristic,
                                boost::predecessor_map(g.predecessors).
                                distance_map(g.distances).
                                vertex_index_map(g.indices).
                                color_map(g.colors).
                                visitor(*astar_visitor));
        }


        //We don't know why boost needs the first template class but IT NEEDS IT. DO NOT REMOVE.
        // Graph : graph_type (ex. directed_graph_type)
        // PRX_Graph : graph_t (ex. directed_graph_t)
        // Vertex : the vertex index type (ex. directed_vertex_index_t)
        // Heuristic: the heuristic type that you want to use
        // Visitor : Visitor's type that you will use

        /**
         * A wrapper around the Boost A* search.
         * @param g A pointer to a graph to search.
         * @param start The start vertex.
         * @param heuristic A pre-constructed heuristic.
         * @param astar_visitor A pre-constructed visitor.
         */
        template<class Graph, class PRX_Graph, class Vertex, class Heuristic, class Visitor>
        void astar_search(PRX_Graph* g, Vertex start, Heuristic* heuristic, Visitor* astar_visitor)
        {

            foreach(Vertex v, boost::vertices(g->graph))
            {
                g->colors[v] = boost::color_traits<boost::default_color_type > ().white();
                g->predecessors[v] = v;
            }
            //        PRX_INFO_S("Performing an A* search with new heuristic and new visitor ... ");    
            boost::astar_search(g->graph, start,
                                *heuristic,
                                boost::predecessor_map(g->predecessors).
                                distance_map(g->distances).
                                vertex_index_map(g->indices).
                                color_map(g->colors).
                                visitor(*astar_visitor));
        }

        //We don't know why boost needs the first template class but IT NEEDS IT. DO NOT REMOVE.
        // Graph : graph_type (ex. directed_graph_type)
        // PRX_Graph : graph_t (ex. directed_graph_t)
        // Vertex : the vertex index type (ex. directed_vertex_index_t)
        // Heuristic: the heuristic type that you want to use
        // Visitor : Visitor's type that you will use

        /**
         * A wrapper around the Boost Breadth First Search.
         * @param g A graph to search.
         * @param start The start vertex.
         * @param goal The goal vertex.
         * @param bfs_visitor A pre-constructed visitor.
         */
        template<class Graph, class PRX_Graph, class Vertex, class Visitor>
        void breadth_first_search(PRX_Graph& g, Vertex start, Vertex goal, Visitor* bfs_visitor)
        {

            foreach(Vertex v, boost::vertices(g.graph))
            {
                g.colors[v] = boost::color_traits<boost::default_color_type > ().white();
                g.predecessors[v] = v;
            }
            boost::breadth_first_search(g.graph, start,
                                        boost::predecessor_map(g.predecessors).
                                        distance_map(g.distances).
                                        vertex_index_map(g.indices).
                                        color_map(g.colors).
                                        visitor(*bfs_visitor));
        }

        /**
         * A wrapper around the Boost Breadth First Search.
         * @param g A pointer to a graph to search.
         * @param start The start vertex.
         * @param bfs_visitor A pre-constructed visitor.
         */
        template<class Graph, class PRX_Graph, class Vertex, class Visitor>
        void breadth_first_search(PRX_Graph* g, Vertex start, Visitor* bfs_visitor)
        {

            foreach(Vertex v, boost::vertices(g->graph))
            {
                g->colors[v] = boost::color_traits<boost::default_color_type > ().white();
                g->predecessors[v] = v;
            }
            boost::breadth_first_search(g->graph, start,
                                        boost::predecessor_map(g->predecessors).
                                        distance_map(g->distances).
                                        vertex_index_map(g->indices).
                                        color_map(g->colors).
                                        visitor(*bfs_visitor));
        }

        /**
         * A wrapper around the Boost Depth First Search
         * @param g A pointer to a graph to search.
         * @param dfs_visitor A pre-constructed visitor.
         */
        template<class Graph, class PRX_Graph, class Vertex, class Edge, class Visitor>
        void depth_first_search(PRX_Graph* g, Visitor* dfs_visitor)
        {

            boost::depth_first_search(g->graph,
                                      boost::predecessor_map(g->predecessors).
                                      distance_map(g->distances).
                                      vertex_index_map(g->indices).
                                      color_map(g->colors).
                                      visitor(*dfs_visitor));
        }

        /**
         * A wrapper around the Boost Depth First Search
         * @param g A pointer to a graph to search.
         * @param dfs_visitor A pre-constructed visitor.
         */
        template<class Graph, class PRX_Graph, class Vertex, class Edge, class Visitor>
        void depth_first_search(const PRX_Graph* g, Visitor* dfs_visitor)
        {

            boost::depth_first_search(g->graph,
                                      boost::predecessor_map(g->predecessors).
                                      distance_map(g->distances).
                                      vertex_index_map(g->indices).
                                      color_map(g->colors).
                                      visitor(*dfs_visitor));
        }

        /**
         * A wrapper around the Boost Dijkstra's shortest paths.
         * @param g A pointer to a graph to search.
         * @param start The start vertex.
         * @param visitor A pre-constructed visitor.
         */
        template<class Graph, class PRX_Graph, class Vertex, class Visitor>
        void dijkstra_shortest_paths(PRX_Graph* g, Vertex start, Visitor* visitor)
        {

            foreach(Vertex v, boost::vertices(g->graph))
            {
                g->colors[v] = boost::color_traits<boost::default_color_type > ().white();
                g->predecessors[v] = v;
            }

            boost::dijkstra_shortest_paths(g->graph, start,
                                           boost::predecessor_map(g->predecessors).
                                           distance_map(g->distances).
                                           vertex_index_map(g->indices).
                                           color_map(g->colors).
                                           visitor(*visitor));
        }



    }
}

#endif	

