/**
 * @file anytime_rrt.hpp
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

#ifndef PRX_ANYTIME_RRT_HPP
#define	PRX_ANYTIME_RRT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/motion_planners/rrt/rrt.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @brief <b> Anytime-RRT</b>
         * 
         * Anytime-RRT
         * 
         * @author Zakary Littlefield
         */
        class anytime_rrt_t : public rrt_t
        {

          public:
            anytime_rrt_t();
            virtual ~anytime_rrt_t();

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
             * @copydoc motion_planner_t::setup() 
             */
            virtual void setup();

            /** 
             * @copydoc motion_planner_t::get_statistics() 
             */
            virtual const util::statistics_t* get_statistics();


          protected:

            bool choose_target();
            void extend_to_target();

            double cost_to_go(sim::state_t* state);
            double cost_to_come(sim::state_t* state);
            double sel_cost(util::tree_vertex_index_t v);
            std::vector<util::tree_vertex_index_t> neighbors(sim::state_t* state);
            void post_solution();

            void purge_tree(util::tree_vertex_index_t v);

            void prune_node(util::tree_vertex_index_t v);



            /**
             * @brief Storage for propagations.
             */
            sim::plan_t plan;

            /**
             * @brief A count of number of visualization images that have been sent to output.
             */
            unsigned img_count;

            double time_elapsed;

            sim::trajectory_t solution_trajectory;
            sim::plan_t solution_plan;
            double solution_cost;
        };

    }
}

#endif	
