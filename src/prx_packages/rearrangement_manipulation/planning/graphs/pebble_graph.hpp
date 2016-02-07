/**
 * @file pebble_graph.hpp
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

#ifndef PRX_PEBBLE_GRAPH_HPP
#define	PRX_PEBBLE_GRAPH_HPP


#include <vector>
#include <set>

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/simulation/state.hpp"
#include "prx/planning/motion_planners/motion_planner_edge.hpp"

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * @author Athanasios Krontiris
             */
            class pebble_node_t : public util::undirected_node_t
            {

              public:
                pebble_node_t();
                pebble_node_t( unsigned index );
                ~pebble_node_t();

                void init_node(const util::space_t* space, const util::space_point_t* new_point, unsigned pose_index);

                const pebble_node_t& operator=( const pebble_node_t& other );
                
                const util::space_t* state_space;
                unsigned position_index;
            };

            /**
             * @author Athanasios Krontiris
             */
            class pebble_edge_t : public plan::motion_planner_edge_t
            {
              public:
                pebble_edge_t();
                pebble_edge_t(const pebble_edge_t& other);
                
                void clear();                
                
                const pebble_edge_t& operator=( const pebble_edge_t& other );
                bool operator<( const pebble_edge_t& other ) const;
                virtual void init_edge(util::undirected_vertex_index_t source, util::directed_vertex_index_t target, std::set<unsigned>& index_constraints, unsigned src, unsigned tgt);
                virtual void add_plan_reversed(sim::plan_t& extra_plan);
                virtual void get_plan(sim::plan_t& computed_plan, unsigned source);
                virtual void get_plan(sim::plan_t& computed_plan, util::undirected_vertex_index_t source);
                
                std::string print_constraints() const;

                util::undirected_vertex_index_t v_source;
                util::undirected_vertex_index_t v_target;
                std::set< unsigned > constraints;
                std::set< unsigned > full_constraints;
                unsigned p_source;
                unsigned p_target;
                bool is_cleared;
                unsigned reaching_point;
                unsigned retracting_point;
            };
        }
    }
}

#endif	


