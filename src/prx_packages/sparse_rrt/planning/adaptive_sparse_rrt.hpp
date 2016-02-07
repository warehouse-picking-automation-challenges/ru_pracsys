/**
 * @file adaptive_sparse_rrt.hpp
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
#include "adaptive_sparse_rrt_graph.hpp"
#include "prx/utilities/statistics/image.hpp"

namespace prx
{
    namespace packages
    {
        namespace sparse_rrt
        {

            /**
             * Adaptive Sparse-RRT
             * @author Zakary Littlefield
             */
            class adaptive_sparse_rrt_t : public plan::rrt_t
            {
                public:
                    adaptive_sparse_rrt_t();
                    virtual ~adaptive_sparse_rrt_t();

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
                    
                    void branch_and_bound();
                    

                protected:   

                    int get_depth(util::tree_vertex_index_t v);

                    double calculate_cost(sim::state_t* start, sim::state_t* goal, double default_cost);

                    /** 
                     * @copydoc planner_t::update_vis_info() const
                     */
                    virtual void update_vis_info() const;

                    /**
                     * @brief Gets a node from the tree.
                     * @param v The index of the node to return.
                     * @return The casted node.
                     */
                    adaptive_sparse_rrt_node_t* get_vertex(util::tree_vertex_index_t v) const;

                    /**
                     * @brief Gets an edge from the tree.
                     * @param e The index of the edge to return.
                     * @return The casted edge.
                     */
                    adaptive_sparse_rrt_edge_t* get_edge(util::tree_edge_index_t e) const;

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
                    bool is_best_goal(util::tree_vertex_index_t v);

                    /**
                     * @brief Remove a vertex from the tree that is a leaf.
                     * @param v The vertex to remove.
                     */
                    void remove_leaf(util::tree_vertex_index_t v);
                    
                    /**
                     * @brief Sort the neighbor list in nearest.
                     */
                    void sort_neighbors(unsigned val);

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

                    /**
                     * @brief The number of points that have been added to the tree.
                     */
                    unsigned point_count;
                    
                    /**
                     * @brief The amount to reduce local delta_near by.
                     */
                    double delta_near_reduction;
                    
                    /**
                     * @brief The amount to reduce local delta_drain by.
                     */
                    double delta_drain_reduction;
                    
                    bool vis_drain;
                    bool vis_near;
                    bool vis_cost;
                    double best_cost;
                    
                    std::vector<util::image_t*> images;
                    std::vector<util::image_t*> near_images;
                    std::vector<util::image_t*> drain_images;
                    

            };

        }
    }
}

#endif
