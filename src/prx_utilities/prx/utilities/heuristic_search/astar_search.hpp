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

#ifndef PRX_ASTAR_SEARCH_HPP
#define PRX_ASTAR_SEARCH_HPP

#include "prx/utilities/heuristic_search/default_open_set.hpp"
#include "prx/utilities/heuristic_search/astar_open_set.hpp"
#include "prx/utilities/heuristic_search/astar_node.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "astar_open_set.hpp"
#include <boost/graph/properties.hpp>
#include <deque>



namespace prx
{
    namespace util
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
         * @author Justin Cardoza
         */
        class astar_search_t
        {

          public:
            astar_search_t();
            astar_search_t(undirected_graph_t *g);
            virtual ~astar_search_t();

            /**
             * Sets the graph to be searched over.
             * 
             * @brief Sets the graph to be searched over.
             * 
             * @param g A pointer to the graph to search.
             */
            virtual void link_graph(undirected_graph_t *g);

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
            virtual bool solve(undirected_vertex_index_t start, const std::vector<undirected_vertex_index_t>& goals );

            /**
             * Perform a query on the graph with a single goal. Optimized for this 
             * case, so if there is only one goal, ideally this version should be used.
             * 
             * @brief Search the graph and terminate as soon as the goal is found.
             * 
             * @param start The start vertex.
             * @param goal The goal vertex.
             * @return True if a path was discovered, false if not.
             */
            virtual bool solve(undirected_vertex_index_t start, undirected_vertex_index_t goal );

            /**
             * Extracts the list of vertices to traverse in order to get from 
             * \ref start to \goal.
             * 
             * @brief Extracts the path from the goal.
             * 
             * @param start The start vertex.
             * @param goal The goal vertex.
             * @param vertices (Out) A deque to store the path vertices in.
             */
            virtual void extract_path(undirected_vertex_index_t start, undirected_vertex_index_t goal, std::deque<undirected_vertex_index_t>& vertices);

            /**
             * Gets the goal vertex index found by the most recent search.
             * @brief Gets the goal vertex index found by the most recent search.
             * @return The index of the found goal vertex.
             */
            undirected_vertex_index_t get_found_goal() const;

            /**
             * The heuristic function. This estimates the remaining distance from 
             * a vertex to the specified goal vertex. Should be overridden in derived 
             * classes to guide the search.
             * 
             * @brief Estimates the remaining distance to the specified goal.
             * 
             * @param current The current vertex to calculate the heuristic for.
             * @param goal The goal vertex.
             * @return The estimated distance to the goal from the current vertex.
             */
            virtual double heuristic(undirected_vertex_index_t current, undirected_vertex_index_t goal) = 0;

            /**
             * Helper function to calculate the minimum heuristic value over a set of 
             * multiple goals, i.e. the distance estimate for the closest goal to a point.
             * 
             * @brief Estimates the distance to the closest goal from a vertex.
             * 
             * @param current The current vertex to calculate the heuristic for.
             * @param goals The list of goal vertices.
             * @return The estimated distance to the closest goal from the current vertex.
             */
            virtual double heuristic(undirected_vertex_index_t current, const std::vector<undirected_vertex_index_t>& goals);

            /**
             * Called when a vertex is initialized. Override this if you need to do any 
             * special handling at that point.
             * 
             * @brief Called when a vertex is initialized.
             * 
             * @param vertex The vertex being initialized.
             */
            virtual void initialize_vertex(undirected_vertex_index_t vertex);

            /**
             * Called when a vertex is discovered. Override this if you need to do any 
             * special handling at that point.
             * 
             * @brief Called when a vertex is discovered.
             * 
             * @param vertex The vertex being discovered.
             */
            virtual void discover_vertex(undirected_vertex_index_t vertex);

            /**
             * Called when a vertex is examined. Override this if you need to do any 
             * special handling at that point.
             * 
             * @brief Called when a vertex is examined.
             * 
             * @param vertex The vertex being examined.
             */
            virtual bool examine_vertex(undirected_vertex_index_t vertex);

            /**
             * Called when a vertex is finished. Override this if you need to do any 
             * special handling at that point.
             * 
             * @brief Called when a vertex is finished.
             * 
             * @param vertex The vertex being finished.
             */
            virtual void finish_vertex(undirected_vertex_index_t vertex);

            /**
             * Called when an edge is examined. Override this if you need to do any 
             * special handling at that point.
             * 
             * @brief Called when a edge is examined.
             * 
             * @param edge The edge being examined.
             */
            virtual bool examine_edge(undirected_vertex_index_t u, undirected_edge_index_t edge, undirected_vertex_index_t v);

            /**
             * Called when an edge is relaxed. Override this if you need to do any 
             * special handling at that point.
             * 
             * @brief Called when a edge is relaxed.
             * 
             * @param edge The edge being relaxed.
             */
            virtual void relaxed_edge(undirected_edge_index_t edge);

            /**
             * Called when an edge is not relaxed. Override this if you need to do any 
             * special handling at that point.
             * 
             * @brief Called when a edge is not relaxed.
             * 
             * @param edge The edge that was not relaxed.
             */
            virtual void not_relaxed_edge(undirected_edge_index_t edge);


          protected:
            undirected_graph_t *graph;
//            default_open_set_t<astar_node_t> openSet;
            astar_open_set_t open_set;
            undirected_vertex_index_t foundGoal;

            static const boost::default_color_type WHITE, GRAY, BLACK;
            
            virtual void openset_insert_node(undirected_vertex_index_t parent, undirected_vertex_index_t v, double f);
        };
    }
}



#endif