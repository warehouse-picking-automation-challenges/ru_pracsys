/**
 * @file informed_rrt_star.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Zakary Littlefield, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_INFORMED_RRT_STAR_HPP
#define	PRX_INFORMED_RRT_STAR_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/motion_planners/rrt/rrt.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @brief <b>Informed RRT*</b>
         * 
         * Informed RRT* 
         * 
         * @author Zakary Littlefield
         */
        class informed_rrt_star_t : public rrt_t
        {

          public:
            informed_rrt_star_t();
            virtual ~informed_rrt_star_t();

            /** 
             * @copydoc motion_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*) 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /** 
             * @copydoc motion_planner_t::resolve_query() 
             */
            virtual void resolve_query();

            /** 
             * @copydoc motion_planner_t::step() 
             */
            virtual void step();

            /** 
             * @copydoc motion_planner_t::get_statistics() 
             */
            virtual const util::statistics_t* get_statistics();

            /**
             * Updates the cost values at nodes in the subtree. This is needed when a better connection is 
             * made in the tree.
             * @brief Updates the cost values at nodes in the subtree.
             * @param v The root of the subtree to update.
             */
            virtual void update_costs(util::tree_vertex_index_t v);


            /** 
             * @copydoc motion_planner_t::resolve_query() 
             */
            virtual void hacked_resolve();

          protected:

            double best_solution_cost;

            /**
             * @brief Updates the k value necessary for "connectivity" in the represented RRG graph. See the related literature for explanations of this parameter.
             */
            void update_k();

            /**
             * @brief The number of neighbors to connect to.
             */
            double k;

            /**
             * @brief Storage for propagations.
             */
            sim::trajectory_t new_traj;

            /**
             * @brief Get the neighboring nodes to a state.
             * @param state The state to find closest nodes from.
             * @return A list of the closest nodes.
             */
            std::vector<util::tree_vertex_index_t> neighbors(sim::state_t* state);

            /**
             * @brief A list of nodes that might be considered goals.
             */
            std::vector<util::tree_vertex_index_t> possible_goals;

            /**
             * @brief A count of number of visualization images that have been sent to output.
             */
            unsigned img_count;

            double time_elapsed;
        };

    }
}

#endif	
