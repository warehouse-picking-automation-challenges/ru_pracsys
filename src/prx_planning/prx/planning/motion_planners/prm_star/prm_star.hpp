/**
 * @file prm_star.hpp
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

#ifndef PRX_PRM_HPP
#define	PRX_PRM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/motion_planners/prm_star/prm_star_graph.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx
{
    namespace plan
    {

        class pno_criterion_t;
        class astar_module_t;
        /**
         * @anchor prm_star_t
         *
         * Probabilistic Roadmap Method (PRM*):
         * A planning method which computes collision-free roadmaps, generally for any
         * system which has a BVP solver.
         *
         * @brief <b> Probabilistic Roadmap Method (PRM*) </b>
         * 
         * @author Athanasios Krontiris
         */
        class prm_star_t : public motion_planner_t
        {

          public:
            prm_star_t();
            virtual ~prm_star_t();

            /**
             * @copydoc motion_planner_t::init(const util::parameter_reader_t*,const util::parameter_reader_t*) 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @copydoc motion_planner_t::reset() 
             */
            virtual void reset();

            /**
             * @copydoc motion_planner_t::link_specification(specification_t*) 
             */
            void link_specification(specification_t* new_spec);
            
            /** 
             * @copydoc motion_planner_t::setup() 
             * 
             * Will occupy memory for the random_point and the new_control, after 
             * planning_query has been linked. 
             */
            virtual void setup();

            /** 
             * @copydoc motion_planner_t::step() 
             * 
             * The algorithm will add one new node for every call of this function.
             */
            virtual void step();

            /** 
             * @copydoc motion_planner_t::resolve_query() 
             *
             * At the end of the resolve_query the algorithm will remove the vertices 
             * for the start and the goal for this specific query from the graph.
             */
            virtual void resolve_query();

            void resolve_query(util::space_t* object_space, world_model_t* model);

            /**
             * @copydoc motion_planner_t::succeeded() const 
             */
            virtual bool succeeded() const;

            /**
             * @copydoc motion_planner_t::serialize() const 
             */
            virtual bool serialize();

            /**
             * @copydoc motion_planner_t::deserialize() const 
             */
            virtual bool deserialize();
            
            virtual int get_number_of_roadmap_vertices()
            {
                return boost::num_vertices(graph.graph);
            }
            
            virtual int get_number_of_roadmap_edges()
            {
                return boost::num_edges(graph.graph);
            }

            /**
             * @copydoc motion_planner_t::get_statistics() const 
             */
            virtual const util::statistics_t* get_statistics();

            friend class pno_criterion_t;
            
          protected:
            /** @brief Temporary path storage. */
            sim::trajectory_t path1;
            /** @brief Temporary path storage. */
            sim::trajectory_t path2;
            /** @brief Vertex index to refer to the last node added to the graph. */
            util::undirected_vertex_index_t v_new;
            /** @brief The planning structure maintained by the PRM. */
            util::undirected_graph_t graph;
            /** @brief Temporary storage for random samples. */
            util::space_point_t* random_point;
            /** @brief Temporary plan storage */
            sim::plan_t new_plan;
            /** @brief Temporary plan storage. */
            sim::plan_t new_plan2;
            /** @brief The number of nearest neighbors to attempt connections with. */
            unsigned int k;
            /** @brief Determines if the prm is delta-prm or k-prm*/
            bool delta_prm;
            /** @brief Determines the radius of the nearest neighbor radial query (delta-prm)*/
            double r_n;
            /** @brief The astar algorithm used for searching the graph*/
            astar_module_t* astar;

            /** @brief The number of edges in the PRM's planning structure. */
            int num_edges;
            /** @brief The number of nodes in the PRM's planning structure. */
            int num_vertices;
            /** @brief The total number of attempted sample generations. */
            int num_generated;
            /** @brief The length of the last generated solution. */
            double last_solution_length;
            /** @brief Whether to be running for Probabilistic Near-Optimality */
            bool pno_mode;

            bool lazy_collision_mode;

            /**
             * @copydoc planner_t::update_vis_info() const
             */
            virtual void update_vis_info() const;

            /**
             * @brief Generates a collision-free random sample and stores it in random_point.
             */
            virtual void valid_random_sample();

            /**
             * Adds a new node in the graph and trying to connect it with the 
             * existing graph. 
             * 
             * @brief Add a new node to the graph.
             *
             * @param n_state The new state that I want to add in the graph.
             * 
             * @return The index for the node in the boost::graph.
             */
            virtual std::pair<bool,util::undirected_vertex_index_t> add_node(const util::space_point_t* n_state);

            /**
             * @brief Connects the node v on the existing graph.
             * 
             * @param v Its the index of an existing node on the graph.
             */
            virtual void connect_node(util::undirected_vertex_index_t v);

            /**
             * @brief Connects the node v in a neighbor of radian rad on the existing graph.
             * 
             * @param v Its the index of an existing node on the graph.
             * @param rad The diameter of the neighbor that we need to check in order to connect the new node on the graph.
             */
            virtual void connect_node(util::undirected_vertex_index_t v, double rad);

            /**
             * @brief Tries to link the node v with the neighbor nodes in the vector neighbors.
             * 
             * @param v Its the index of an existing node on the graph, that we want to connect on the graph.
             * @param neighbors The nodes that we will try to connect with the node v.
             */
            virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);

            /**
             * @brief Updates the k value for the k-prm*.
             * 
             * @param nr_nodes The number of the nodes in the graph.
             */
            virtual void update_k(unsigned nr_nodes);
            
            /**
             * @brief Checks if the trajectory on the edge is valid or not. 
             * 
             * Checks if the trajectory on the edge is valid or not. 
             * 
             * @param path The trajectory that has to be checked.
             */
            bool is_valid_trajectory(const sim::trajectory_t& path);
            
            /** @brief A flag indicating whether the start and goal points will be checked for collisions before added in the graph*/
            bool no_collision_query_type;
            /** @brief If the flag is true then the prm will select a point close to the given start and goal */
            bool near_query_type;
            /** @brief A flag indicating whether PRM should send its computed graph to the visualization node. */
            bool visualize_graph;
            /** @brief A flag indicating whether PRM should send computed solutions to the visualization node. */
            bool visualize_solutions;
            /** @brief A unique name identifier for the PRM's graph. */
            std::string visualization_graph_name;
            /** @brief A unique name identifier for the PRM's solution path. */
            std::string visualization_solutions_name;
            /** @brief A list of all bodies which the PRM wishes to visualize. */
            std::vector<std::string> visualization_bodies;
            /** @brief A list of colors to use for displaying solution paths. */
            std::vector<std::string> solutions_colors;
            /** @brief The color to visualize the PRM graph in. */
            std::string graph_color;
            int execute_num;
            bool serialize_plan;

            /** @brief Threshold for similar nodes.*/
            double similarity_threshold;
            /** @brief Checks if we need to remove the start node after we finish with the query.*/
            bool remove_start;
            /** @brief Checks if we need to remove the goal nodes after we finish with the query.*/
            std::vector<bool> remove_goals;

        };

    }
}

#endif
