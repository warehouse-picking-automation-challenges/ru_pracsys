/**
 * @file space_grid.hpp
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

#ifndef PRX_SPACE_GRID_HPP
#define	PRX_SPACE_GRID_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "reachable_sparse_rrt_graph.hpp"
#include "prx/simulation/control.hpp"
#include "prx/simulation/plan.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"


namespace prx
{
    namespace packages
    {
        namespace sparse_rrt
        {

            struct cell
            {
              reachable_sparse_rrt_node_t* node;
              double time;
              int overlap;
            };

            /**
             * @brief A grid that divides the space
             * 
             * A division of a space with a grid. 
             * 
             * @author Zakary Littlefield
             */
            class space_grid_t
            {
              public:
              	space_grid_t(std::vector<util::bounds_t* >& in_bounds,std::vector<unsigned> in_divisions);
                ~space_grid_t();

              	cell* get(std::vector<unsigned>& position);

              	cell* get(util::space_point_t* point);

                unsigned get_index(std::vector<unsigned>& position);
                unsigned get_flat_index(util::space_point_t* point);
                unsigned get_flat_index(std::vector<double>& point);

                void give_validity_checker(plan::validity_checker_t* check, util::space_point_t* point, bool checking = false);

                void add_node(reachable_sparse_rrt_node_t* node, const util::space_t* control_space, double simulation_step = .002, unsigned num_sim_steps = 200);

                void recompute_grid();
                unsigned total_occupied;
                double grid_weight;

                void intersect(space_grid_t* other_grid);

                std::vector<util::bounds_t*> bounds;
                std::vector<unsigned> divisions;

                bool in_bounds(util::space_point_t* point);
                bool in_bounds(std::vector<double>& point);

                void get_controls(util::space_point_t* goal_point, sim::plan_t& plan);

                void random_selection(sim::plan_t& plan);

// #ifdef OCTAVE_FOUND
//                 static util::octave_caller_t* caller;
// #endif                
                std::vector<util::bounds_t*> point_bounds;
              protected:
                void get_controls(unsigned index, sim::plan_t& plan, util::space_point_t* goal_point = NULL);
                bool collision_check;
                plan::validity_checker_t* checker;
                util::space_point_t* test_point;
                const util::space_t* ctrl_space;
                std::vector<util::bounds_t*> intersection;
                std::vector<unsigned> temp_index;
              	unsigned dim;
              	std::vector<unsigned> multiplier;
              	std::vector<double> cell_size;
                std::vector<double> min_val;
              	cell** grid;
                unsigned number_of_cells;
                reachable_sparse_rrt_node_t* base_node;

                //stuff for the integration
                double* level_set;
                std::vector<sim::control_t*> control_field;
                double** xs;

                double max_time;
                double simulation_step;
                std::vector<unsigned> random_indices;
                double old_cost;
                cell default_cell;

            };


        }
    }
}

#endif