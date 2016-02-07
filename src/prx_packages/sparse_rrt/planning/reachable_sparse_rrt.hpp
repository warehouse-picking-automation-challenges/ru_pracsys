/**
 * @file reachable_sparse_rrt.hpp
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

#ifndef PRX_SPARSE_RRT_HPP
#define	PRX_SPARSE_RRT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/motion_planners/rrt/rrt.hpp"
#include "reachable_sparse_rrt_graph.hpp"
#include "space_grid.hpp"

namespace prx
{
    namespace packages
    {
        namespace sparse_rrt
        {
            /**
             * Sparse-RRT
             * @author Zakary Littlefield
             */
            class reachable_sparse_rrt_t : public plan::rrt_t
            {
                public:
                    reachable_sparse_rrt_t();
                    virtual ~reachable_sparse_rrt_t();

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

                protected:
                    /**
                     * @copydoc planner_t::update_vis_info() const
                     */
                    virtual void update_vis_info() const;

                    void init_grid(util::tree_vertex_index_t v);

                    void compute_reachability(util::tree_vertex_index_t v);


                    // void create_reach_region(util::tree_vertex_index_t v);

                    /**
                     * @brief Gets a node from the tree.
                     * @param v The index of the node to return.
                     * @return The casted node.
                     */
                    reachable_sparse_rrt_node_t* get_vertex(util::tree_vertex_index_t v) const;

                    /**
                     * @brief Gets an edge from the tree.
                     * @param e The index of the edge to return.
                     * @return The casted edge.
                     */
                    reachable_sparse_rrt_edge_t* get_edge(util::tree_edge_index_t e) const;

                    /**
                     * Returns the nearest vertex to the given state.
                     * @param state The state to find the closest node to.
                     * @return The index to the closest node in the tree.
                     */
                    util::tree_vertex_index_t nearest_vertex();

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
                    bool is_best_goal(util::tree_vertex_index_t v);

                    /**
                     * @brief Remove a vertex from the tree that is a leaf.
                     * @param v The vertex to remove.
                     */
                    void remove_leaf(util::tree_vertex_index_t v);

                    void insert_into_list();

                    void update_list(unsigned index);

                    void initialize_goal_path();

                    void reinit_list(util::tree_vertex_index_t v);

                    void recompute_subtree(util::tree_vertex_index_t v, double update_val);

                    /**
                     * @brief An index to the best goal node we have found.
                     */
                    util::tree_vertex_index_t best_goal;

                    /**
                     * @brief A real solution has been found.
                     */
                    bool real_solution;

                    /**
                     * @brief The output directory to print images to.
                     */
                    std::string output_directory;

                    /**
                     * @brief The number of points that have been added to the tree.
                     */
                    unsigned point_count;

                    double total_weight;
                    unsigned reach_steps;

                    double intersection_radius;

                    sim::plan_t plan;
                    sim::plan_t plan2;

                    std::vector<unsigned> divisions;
                    std::vector<double> cell_sizes;
                    std::vector<double> min_bound;
                    std::vector<double> max_bound;

                    unsigned num_trajs;

                    std::vector<sim::trajectory_t*> trajs;

                    std::deque<util::tree_vertex_index_t> random_selection;
                    std::deque<double*> random_selection_cells;

                    unsigned img_count;

                    double thresh;
                    sim::trajectory_t additional_traj;





            };

        }
    }
}

#endif
