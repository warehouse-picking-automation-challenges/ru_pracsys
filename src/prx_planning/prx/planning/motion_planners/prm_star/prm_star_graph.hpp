/**
 * @file prm_star_graph.hpp
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

#ifndef PRX_PRM_STAR_GRAPH_HPP
#define	PRX_PRM_STAR_GRAPH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @anchor prm_star_node_t
         *
         * PRM nodes must remember the state space point they represent.  These nodes
         * additionally carry extra information for PRM and its variants.
         *
         * @brief <b> Node class used by the PRM planning structure. </b>
         *
         * @author Athanasios Krontiris
         */
        class prm_star_node_t : public util::undirected_node_t
        {

          public:

            ~prm_star_node_t(){ }

            /**
             * @brief Initialize the node with required space and point information.
             *
             * @param space The state space this node exists in.
             * @param new_point The point in the space where this node exsists.
             */
            void init_node(const util::space_t* space, const util::space_point_t* new_point)
            {
                point = space->clone_point(new_point);
            }

            /**
             * @brief Output node information to a stream.
             *
             * @param output_stream The stream to which to serialize the node information.
             * @param point_space The space where the space point lies. TODO : Do we really need this?
             */
            void serialize(std::ofstream& output_stream, const util::space_t* point_space)
            {
                util::undirected_node_t::serialize(output_stream, point_space);
                output_stream << " " << trusted;
            }

            /**
             * @brief Read in node information from a stream.
             *
             * @param input_stream The stream from which to read the node information.
             * @param point_space The space where the space point lies. TODO : Do we really need this?
             */
            void deserialize(std::ifstream& input_stream, const util::space_t* point_space)
            {
                util::undirected_node_t::deserialize(input_stream, point_space);
                input_stream >> trusted;
            }
            
            /** @brief If the node is outside the shelf and it is definitely safe.*/
            bool trusted;
        };

        /**
         * @anchor prm_star_edge_t
         *
         * Edges in the PRM planning structure which must hold information such as weights,
         * controls used for generation, and an identifier.
         *
         * @brief <b> Edge class used in the PRM planning structure. </b>
         *
         * @author Athanasios Krontiris
         */
        class prm_star_edge_t : public motion_planner_edge_t
        {

          public:

            prm_star_edge_t()
            {
                trusted = false;
                checked = -1;
            }

            ~prm_star_edge_t(){ }

            void add_plan(const util::space_t* space, const sim::plan_t& new_plan)
            {
                plan.link_control_space(space);
                plan = new_plan;
            }

            bool is_checked(int run_id)
            {
                return run_id == checked;
            }
            
            /**
             * @brief Output edge information to a stream.
             *
             * @param output_stream The stream to which to serialize the node information.
             */
            virtual void serialize(std::ofstream& output_stream)
            {
                util::undirected_edge_t::serialize(output_stream);
                output_stream << " " << trusted;
            }

            /**
             * @brief Read in edge information to a stream.
             *
             * @param input_stream The stream from which to read the node information.
             */
            virtual void deserialize(std::ifstream& input_stream)
            {
                util::undirected_edge_t::deserialize(input_stream);
                input_stream >> trusted;
                checked = -1;
            }

            /** @brief The cost associated with this edge. */
            double cost;

            /** @brief If the edge is outside the shelf and it is definitely safe.*/
            bool trusted;

            /** @brief Instead of restarting the boolean value if an edge is checked or not, 
             *  a counter will tell us if this edge is checked during this run.
             */
            int checked;
        };

        /**
         * @anchor prm_star_found_goal
         *
         * This class is used by the boost::astar function.  It's purpose is to identify
         * when the goal node has been expanded.
         *
         * @brief <b> Boost goal identification function for A* searches. </b>
         *
         * @author Athanasios Krontiris
         */
        struct prm_star_found_goal
        {

            /** @brief Vertex index of the goal node. */
            util::undirected_vertex_index_t v_goal;

            /**
             * @brief Function used by boost::astar to determine if the goal has been expanded.
             *
             * @param v The node to make the goal.
             */
            prm_star_found_goal(util::undirected_vertex_index_t v)
            {
                v_goal = v;
            }
        };

        /**
         * @anchor default_prm_star_distance_heuristic
         *
         * This class is a default heuristic to be used by A* searches for PRM.
         *
         * @brief <b> Distance heuristic used by PRM A* searches. </b>
         *
         * @author Athanasios Krontiris
         */
        class default_prm_star_distance_heuristic : public boost::astar_heuristic<util::undirected_graph_type, double>
        {

          public:

            /**
             * Constructor for the heuristic. Initializes the distance metric for the heuristic.
             * 
             * @brief Constructor for the heuristic.
             *
             * @param metric The distance metric to be used to determine distances in the space.
             */
            default_prm_star_distance_heuristic(util::distance_metric_t* metric)
            {
                distance_metric = metric;
            }

            /**
             * Constructor for the heuristic. Initializes the graph and the distance metric for the 
             * heuristic.
             * 
             * @brief Constructor for the heuristic.
             *
             * @param g The graph built by PRM to link to this heuristic.
             * @param metric The distance metric to be used to determine distances in the space.
             */
            default_prm_star_distance_heuristic(const util::undirected_graph_t * g, util::distance_metric_t* metric)
            {
                inner_graph = g;
                distance_metric = metric;
            }

            /**
             * @brief Constructor for the heuristic. Initializes the graph the distance metric and 
             * a set of goals for the heuristic.
             *
             * @param g The graph built by PRM to link to this heuristic.
             * @param goals The set of goals from which to draw the heuristic.
             * @param metric The distance metric to be used to determine distances in the space.
             */
            default_prm_star_distance_heuristic(const util::undirected_graph_t * g, std::vector<util::undirected_vertex_index_t>& goals, util::distance_metric_t* metric)
            {
                inner_graph = g;
                distance_metric = metric;
                set_new_goals(goals);
            }

            /**
             * Sets a new graph and a new set of goals.
             * 
             * @brief Sets a new graph and a new set of goals.
             * 
             * @param g The graph built by PRM to link to this heuristic.
             * @param goals The new set of goals.
             */
            void set(const util::undirected_graph_t * g, std::vector<util::undirected_vertex_index_t>& goals)
            {
                inner_graph = g;

                foreach(util::undirected_vertex_index_t goal, goals)
                {
                    goal_nodes.push_back(g->operator[](goal));
                }
            }

            /**
             * Sets new set of goals.
             * 
             * @brief Sets new set of goals.
             * 
             * @param goals The new set of goals.
             */
            void set_new_goals(std::vector<util::undirected_vertex_index_t>& goals)
            {

                foreach(util::undirected_vertex_index_t goal, goals)
                {
                    goal_nodes.push_back(inner_graph->operator[](goal));
                }
            }

            /**
             * Sets new distance metric for the heuristic. 
             * 
             * @brief Sets new distance metric for the heuristic. 
             * 
             * @param metric The new distance metric that the heuristic has to use now.
             */
            void set_new_metric(util::distance_metric_t* metric)
            {
                distance_metric = metric;
            }

            /**
             * @brief Calling operator for the heuristic.
             *
             * @param u The vertex for which to compute the heuristic.
             *
             * @return The heuristic value for u.
             */
            double operator()(Vertex u)
            {
                double dist = PRX_INFINITY;
                double new_dist;
                util::undirected_node_t* new_node = inner_graph->operator[](u);

                foreach(util::undirected_node_t* goal_node, goal_nodes)
                {
                    new_dist = distance_metric->distance_function(new_node->point, goal_node->point);
                    if( new_dist < dist )
                        dist = new_dist;
                }
                return dist;
            }

          private:
            /** @brief The set of goal nodes for the heuristic. */
            std::vector<util::undirected_node_t*> goal_nodes;
            /** @brief The distance metric to use to determine distance. */
            util::distance_metric_t* distance_metric;
            /** @brief The graph over which this heuristic is operating. */
            const util::undirected_graph_t * inner_graph;
        };

        /**
         * @anchor default_prm_star_astar_visitor
         *
         * Boost astar visitor class used by PRM by default.
         *
         * @brief <b> Visitor class used in PRM for A* searches. </b>
         *
         * @author Athanasios Krontiris
         */
        class default_prm_star_astar_visitor : public boost::default_astar_visitor
        {

          public:

            /**     
             * Default prm astar visitor constructor.
             * 
             * @brief Default prm astar visitor constructor.
             */
            default_prm_star_astar_visitor()
            {
                m_goals = NULL;
            }

            /**
             * Sets new set of goals in the visitor. 
             * 
             * @brief Sets new set of goals in the visitor. 
             * 
             * @param goals The new set of goals.
             */
            void set_new_goals(const std::vector<util::undirected_vertex_index_t>* goals)
            {
                m_goals = goals;
            }

            /**
             * Vertex examination function called when a node is expanded.
             * 
             * @brief Vertex examination function called when a node is expanded.
             *
             * @param u The vertex to examine.
             * @param g The graph to which this vertex belongs.
             *
             * @note Throws a prm_star_found_goal exception if u is the goal.
             */
            template <class Graph_t>
            void examine_vertex(util::undirected_vertex_index_t u, Graph_t& g)
            {
                PRX_ASSERT(m_goals != NULL);

                foreach(util::undirected_vertex_index_t m_goal, *m_goals)
                {
                    if( u == m_goal )
                        throw prm_star_found_goal(u);
                }
            }

          private:
            /** @brief The stored set of goals for this A* search. */
            const std::vector<util::undirected_vertex_index_t>* m_goals;
        };


    }
}

#endif
