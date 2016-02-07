/**
 * @file planning_astar_search.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_A_STAR_HPP
#define PRX_A_STAR_HPP

#include "prx/utilities/heuristic_search/astar_search.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/plan.hpp"


#include <boost/graph/properties.hpp>
#include <deque>

#include <pluginlib/class_loader.h>


namespace prx
{
    namespace plan
    {

        /**
         * @anchor astar_module_t
         * 
         * Flexible implementation of the A* heuristic graph search algorithm. To use, 
         * derive from this class and provide an implementation of the single-goal 
         * heuristic function. The callback functions can also be overridden to give 
         * fine-grained information and control over the search process.
         * 
         * @brief <b> Flexible implementation of A* search. </b>
         * 
         * @author Athanasios Krontiris
         */
        class astar_module_t : public util::astar_search_t
        {

          public:

            enum astar_mode
            {

                PRX_NO_EDGE_INFO, PRX_REUSE_EDGE_INFO, PRX_REPROPAGATE_EDGE_INFO
            };

            virtual ~astar_module_t();

            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
            virtual void restart();

            virtual void link_graph(util::undirected_graph_t *g);
            virtual void link_spaces(const util::space_t* state_sp, const util::space_t* control_sp);
            virtual void link_distance_metric(util::distance_metric_t* m);
            virtual void link_validity_checker(validity_checker_t* checker);
            virtual void link_local_planner(local_planner_t* loc_planner);

            virtual void link_modules(validity_checker_t* checker, local_planner_t* loc_planner);
            virtual void setup_astar_module(util::undirected_graph_t *g, const util::space_t* state_sp, const util::space_t* control_sp, validity_checker_t* checker, local_planner_t* loc_planner);


            virtual void set_astar_mode(astar_mode mode);
            virtual void block_edge(util::undirected_edge_index_t e);

            virtual bool examine_edge(util::undirected_vertex_index_t u, util::undirected_edge_index_t edge, util::undirected_vertex_index_t v);

            /**
             * @brief Retrieve the loader for this class instance.
             *
             * @return The pluginlib class loader for creating instances of planners.
             */
            static pluginlib::ClassLoader<astar_module_t>& get_loader();

          protected:
            const util::space_t* state_space;
            const util::space_t* control_space;
            validity_checker_t* validity_checker;
            local_planner_t* local_planner;
            util::distance_metric_t* metric;

            std::vector<util::undirected_edge_index_t> blocked_edges;

            sim::trajectory_t edge_path;
            sim::plan_t edge_plan;

            astar_mode solve_mode;


          private:
            /** @brief The pluginlib loader which is returned by the get_loader() class. */
            static pluginlib::ClassLoader<astar_module_t> loader;
        };
    }
}



#endif