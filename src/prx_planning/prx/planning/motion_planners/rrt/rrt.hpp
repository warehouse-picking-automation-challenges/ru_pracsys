/**
 * @file rrt.hpp
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

#ifndef PRX_RRT_HPP
#define	PRX_RRT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/motion_planners/rrt/rrt_graph.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @brief <b> Randomly-exploring Random Tree </b>
         * @author Zakary Littlefield
         */
        class rrt_t : public motion_planner_t
        {
          friend class rrtc_t;

          public:
            rrt_t();
            virtual ~rrt_t();

            /** 
             * @copydoc motion_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*)  
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /** 
             * @copydoc motion_planner_t::reset() 
             */
            virtual void reset();

            /** 
             * @copydoc motion_planner_t::setup() 
             */
            virtual void setup();

            /** 
             * @copydoc motion_planner_t::step() 
             */
            virtual void step();

            /** 
             * @copydoc motion_planner_t::resolve_query() 
             */
            virtual void resolve_query();

            /** 
             * @copydoc motion_planner_t::succeeded() const 
             */
            virtual bool succeeded() const;

            /** 
             * @copydoc motion_planner_t::serialize() 
             */
            virtual bool serialize();

            /** 
             * @copydoc motion_planner_t::deserialize() 
             */
            virtual bool deserialize();

            /** 
             * @copydoc motion_planner_t::get_statistics() 
             */
            virtual const util::statistics_t* get_statistics();

          protected:

            /**
             * @brief Get the \ref rrt_node_t from the tree.
             * @param v The index of the node to return.
             * @return A casted pointer to the node.
             */
            rrt_node_t* get_vertex(util::tree_vertex_index_t v) const;

            /**
             * @brief Get the \ref rrt_edge_t from the tree.
             * @param e The index of the edge to return.
             * @return A casted pointer to the edge.
             */
            rrt_edge_t* get_edge(util::tree_edge_index_t e) const;

            /**
             * Returns the nearest vertex to the given state.
             * @param state The state to find the closest node to.
             * @return The index to the closest node in the tree.
             */
            util::tree_vertex_index_t nearest_vertex(sim::state_t* state) const;

            /**
             * @brief The actual tree.
             */
            util::tree_t tree;

            /**
             * @brief An index to the root node.
             */
            util::tree_vertex_index_t start_vertex;

            /**
             * @brief A temporary storage for sampled points.
             */
            sim::state_t* sample_point;

            /**
             * @brief Clock for collecting timing statistics.
             */
            util::sys_clock_t clock;

            /**
             * @brief Keeps the count of iterations.
             */
            int iteration_count;

            /** 
             * @copydoc planner_t::update_vis_info() const
             */
            virtual void update_vis_info() const;

            /**
             * @brief Flag to visualize tree.
             */
            bool visualize_tree;
            /**
             * @brief Flag to visualize the solution.
             */
            bool visualize_solution;
            /**
             * @brief Flag that determines if goals are found within a radius.
             */
            bool radius_solution;

            /**
             * @brief Flag that turns on and off collision checking.
             */
            bool collision_checking;

            /**
             * @brief Flag for goal biasing.
             */
            bool goal_biased;

            /**
             * @brief Flag for using the boost random.
             */
            bool use_boost_random;

            /**
             * @brief The goal biasing percentage.
             */
            double goal_bias_rate;

            /**
             * @brief The visualization name for the tree visualization geometries.
             */
            std::string visualization_tree_name;

            /**
             * @brief The visualization name for the solution visualization geometries.
             */
            std::string visualization_solution_name;

            /**
             * @brief The rigid body that determines visualization positions.
             */
            std::string visualization_body;

            /**
             * @brief The color of the solution path.
             */
            std::string solution_color;

            /**
             * @brief The color of the tree.
             */
            std::string graph_color;

            /**
             * @brief A trajectory to store the result of propagations.
             */
            sim::trajectory_t trajectory;

            /**
             * @brief Optimizes memory allocations by storing a large set of allocated points.
             */
            std::vector<sim::state_t*> pre_alloced_points;
            /**
             * @brief The current max number of points.
             */
            unsigned max_points;

            /**
             * @brief In replanning, this list of colors will visualize solution paths differently.
             */
            std::vector<std::string> color_map;

            /**
             * @brief Index into \c color_map
             */
            int solution_number;

            /**
             * @brief Counter for number of points added to the structure.
             */
            unsigned point_number;
        };

    }
}

#endif
