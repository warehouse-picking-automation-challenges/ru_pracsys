/**
 * @file rrt_star_with_shooting.hpp
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

#ifndef PRX_RRT_STAR_SHOOTING_HPP
#define	PRX_RRT_STAR_SHOOTING_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/motion_planners/rrt/rrt.hpp"


namespace prx
{
    namespace packages
    {
        namespace rrt_star_with_shooting
        {

            /**
             * @brief Randomly-exploring Random Tree* using shooting.
             * 
             * @author Zakary Littlefield
             */
            class rrt_star_with_shooting_t : public plan::rrt_t
            {
                public:
                    rrt_star_with_shooting_t();
                    virtual ~rrt_star_with_shooting_t();

                    /**
                     * @copydoc motion_planner_t::setup();
                     */
                    virtual void setup();

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
                     * @copydoc motion_planner_t::resolve_query() 
                     */
                    virtual void resolve_query();

                protected:

                    /**
                     * Repropagates a subtree. This is required when a "connection" is made to 
                     * make the rest of the tree valid with the new connection.
                     * 
                     * @brief Repropagates a subtree.
                     * @param v The root of the tree to repropagate.
                     */
                    void repropagate(util::tree_vertex_index_t v);

                    /**
                     * Deletes a subtree and the node itself. This is the result when 
                     * a repropagation results in invalid states (such as collisions).
                     * 
                     * @brief Deletes a subtree and the node itself.
                     * @param v The vertex that will be delete along with its children.
                     */
                    void delete_children_and_self(util::tree_vertex_index_t v);

                    /**
                     * @brief Updates the k value necessary for "connectivity" in the represented RRG graph. See the related literature for explanations of this parameter.
                     */
                    void update_k();

                    /**
                     * @brief The number of neighbors to connect to.
                     */
                    double k;

                    /**
                     * @brief Flag denoting when a goal is found.
                     */
                    bool goal_found;

                    /**
                     * @brief A threshold of how close a node must get in order for connection to be valid.
                     */
                    double near_threshold;

                    /**
                     * @brief Storage for propagations.
                     */
                    sim::trajectory_t new_traj;

                    /** @brief Storage for controls.
                     */
                    sim::plan_t plan;

                    /**
                     * @brief Used for rewiring.
                     */
                    sim::plan_t new_plan;

                    /**
                     * @brief Storage for output from distance metrics
                     */
                    std::vector<const util::abstract_node_t*> radial;

                    /**
                     * @brief Get the neighboring nodes to a state.
                     * @param state The state to find closest nodes from.
                     * @return A list of the closest nodes.
                     */
                    void neighbors(sim::state_t* state);

                    /**
                     * @brief A temporary storage of nodes to delete. 
                     */
                    std::vector<util::tree_vertex_index_t> to_delete;

                    /**
                     * @brief A count of number of visualization images that have been sent to output.
                     */
                    unsigned img_count;

                    /**
                     * @brief A holder for the best goal.
                     */
                    util::tree_vertex_index_t best_goal;

                    /**
                     * @brief Storage for nearest neighbors.
                     */
                    std::vector<util::tree_vertex_index_t> Z_near;
            };
        }
    }
}
#endif
