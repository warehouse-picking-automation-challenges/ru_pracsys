/**
 * @file isst.hpp
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

#ifndef PRX_ISST_HPP
#define	PRX_ISST_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/rrt/rrt.hpp"
#include "prx/planning/motion_planners/sst/sst_graph.hpp"

namespace prx
{
    namespace plan
    {

        typedef boost::function<double (const util::space_point_t*)> heuristic_function_t;


        class informed_sample_point_t : public rrt_node_t
        {
          public:
            informed_sample_point_t()
            {
                set = false;
                point = NULL;
                visiting_nodes.clear();
                h_value = 0;
            }
            ~informed_sample_point_t()
            {
                visiting_nodes.clear();
            }
            bool set;
            util::tree_vertex_index_t memory;
            std::vector<util::tree_vertex_index_t> visiting_nodes;
            double h_value;
        };
        /**
         * Stable Sparse-RRT
         * @author Zakary Littlefield
         */
        class isst_t : public plan::rrt_t
        {
            public:
                isst_t();
                virtual ~isst_t();

                /** 
                 * @copydoc motion_planner_t::init(const parameter_reader_t*, const parameter_reader_t*) 
                 */
                virtual void init(const util::parameter_reader_t* reader,const util::parameter_reader_t* template_reader);

                /** 
                 * @copydoc motion_planner_t::step() 
                 */     
                virtual void step();

                /** 
                 * @copydoc motion_planner_t::get_statistics() 
                 */
                virtual const util::statistics_t* get_statistics(); 

                /**
                 * @copydoc motion_planner_t::setup()
                 */
                virtual void setup();

                /** 
                 * @copydoc motion_planner_t::resolve_query() 
                 */
                virtual void resolve_query();

                virtual void update_vis_info() const;
                
                heuristic_function_t h;
                /**
                 * @copydoc motion_planner_t::compute_cost() 
                 */
                virtual double compute_cost();

            protected:   

                void prune_node(util::tree_vertex_index_t v);

                void purge_nodes();

                /**
                 * @brief Gets a node from the tree.
                 * @param v The index of the node to return.
                 * @return The casted node.
                 */
                sst_node_t* get_vertex(util::tree_vertex_index_t v) const;

                /**
                 * @brief Gets an edge from the tree.
                 * @param e The index of the edge to return.
                 * @return The casted edge.
                 */
                sst_edge_t* get_edge(util::tree_edge_index_t e) const;

                /**
                 * Returns the nearest vertex to the given state.
                 * @param state The state to find the closest node to.
                 * @return The index to the closest node in the tree.
                 */
                util::tree_vertex_index_t nearest_vertex(sim::state_t* state);

                /**
                 * @brief Get the neighboring nodes to a state.
                 * @param state The state to find closest nodes from.
                 * @return A list of the closest nodes.
                 */
                std::vector<util::tree_vertex_index_t> neighbors(sim::state_t* state);

                /**
                 * @brief Check the tree if a vertex is a leaf or not.
                 * @param v The vertex index to check.
                 * @return True if leaf, false if not.
                 */
                bool is_leaf(util::tree_vertex_index_t v);

                /**
                 * @brief Tell if a vertex is part of the best path to the goal.
                 * @param v The vertex to test.
                 * @return True if on path to goal, false if not.
                 */
                bool is_best_goal(util::tree_vertex_index_t v) const;

                /**
                 * @brief Remove a vertex from the tree that is a leaf.
                 * @param v The vertex to remove.
                 */
                void remove_leaf(util::tree_vertex_index_t v);

                /**
                 * @brief Remove an entire subtreee.
                 * @param v The start vertex to remove.
                 */
                void remove_subtree(util::tree_vertex_index_t v);

                double heuristic(const util::space_point_t* s);
                double fake_heuristic(const util::space_point_t* s);
                double heuristic(util::tree_vertex_index_t s);

                /**
                 * @brief The radius for pruning, \Delta_drain
                 */
                double delta_drain;

                /**
                 * @brief The number of attempts to propagate outside \c delta
                 */
                int max_attempts;     

                /**
                 * @brief Flag for performing the BestNearest operation.
                 */
                bool radius_nearest;

                /**
                 * @brief Flag for performing the Drain operation.
                 */
                bool drain;

                /**
                 * @brief The radius for bestNearest.
                 */
                double delta_near;

                /**
                 * @brief An index to the best goal node we have found.
                 */
                util::tree_vertex_index_t best_goal;

                /**
                 * @brief A real solution has been found.
                 */
                bool real_solution;

                /**
                 * @brief A counter for output image names.
                 */
                unsigned img_count;

                /**
                 * @brief Storage for output from distance metrics
                 */
                std::vector<const util::abstract_node_t*> radial;

                /**
                 * @brief A counter of failures to propagate outside \c delta
                 */
                unsigned count_of_failure;

                /**
                 * @brief The output directory to print images to.
                 */
                std::string output_directory;

                /**
                 * @brief The cost threshold for white pixels in images.
                 */
                double thresh;

                bool steering;

                double time_elapsed;

                int last_stat;

                bool rrt_first;

                bool use_heuristic;
                bool complex_heuristic;
                bool branch_and_bound;
                bool use_trrt_heuristic;
                bool sst_only;
                double first_solution_time;

                /**
                * @brief Temperature for traversing cost maps.
                */
                double current_temp;
                double temp_rate;
                double min_cost;
                double max_cost;

                /** @brief Store random points. */
                util::tree_t sample_graph;

                util::distance_metric_t* sample_metric;


                unsigned out_drain;
                unsigned valid;
                unsigned h_remove;
                unsigned bnb_remove;
                unsigned replaced;


        };

    }
}

#endif
