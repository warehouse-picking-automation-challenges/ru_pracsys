/**
 * @file pebble_graph.cpp
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

#include <vector>
#include <set>

#include "planning/graphs/pebble_graph.hpp"

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {
            // ==============
            //  Pebble Node
            // ==============

            pebble_node_t::pebble_node_t()
            {
                state_space = NULL;
                point = NULL;
                position_index = -1;

            }

            pebble_node_t::pebble_node_t(unsigned index)
            {
                state_space = NULL;
                point = NULL;
                position_index = index;
            }

            pebble_node_t::~pebble_node_t() { }

            void pebble_node_t::init_node(const util::space_t* space, const util::space_point_t* new_point, unsigned pose_index)
            {
                state_space = space;
                point = state_space->clone_point(new_point);
                position_index = pose_index;
            }

            const pebble_node_t& pebble_node_t::operator=(const pebble_node_t& other)
            {
                state_space = other.state_space;
                position_index = other.position_index;
                point = state_space->clone_point(other.point);
                return *this;
            }


            // ==============
            //  Pebble Edge
            // ==============

            pebble_edge_t::pebble_edge_t()
            {
                is_cleared = false;
                v_source = NULL;
                v_target = NULL;
                p_source = -1;
                p_target = -1;
                plan.clear();
                path.clear();
                constraints.clear();
            }

            pebble_edge_t::pebble_edge_t(const pebble_edge_t& other)
            {
                *this = other;
                is_cleared = false;
            }

            void pebble_edge_t::clear()
            {
                is_cleared = true;
                v_source = NULL;
                v_target = NULL;
                p_source = -1;
                p_target = -1;
                plan.clear();
                path.clear();
                constraints.clear();
            }

            const pebble_edge_t& pebble_edge_t::operator=(const pebble_edge_t& other)
            {
                constraints = other.constraints;
                p_source = other.p_source;
                p_target = other.p_target;
                v_source = other.v_source;
                v_target = other.v_target;

                plan = other.plan;
                path = other.path;

                return *this;
            }

            void pebble_edge_t::init_edge(util::undirected_vertex_index_t source, util::directed_vertex_index_t target, std::set<unsigned>& index_constraints, unsigned src, unsigned tgt)
            {
                v_source = source;
                v_target = target;
                constraints = index_constraints;
                p_source = src;
                p_target = tgt;
            }

            void pebble_edge_t::add_plan_reversed(sim::plan_t& extra_plan)
            {

                for( int i = (int)extra_plan.size() - 1; i >= 0; --i )
                {
                    plan.copy_onto_back(extra_plan[i].control, extra_plan[i].duration);
                }

            }

            void pebble_edge_t::get_plan(sim::plan_t& computed_plan, unsigned source)
            {
                if( p_source == source )
                    computed_plan += plan;
                else
                {
                    PRX_ASSERT(p_target == source);
                    for( int i = (int)plan.size() - 1; i >= 0; --i )
                    {
                        computed_plan.copy_onto_back(plan[i].control, plan[i].duration);

                    }
                }
            }

            void pebble_edge_t::get_plan(sim::plan_t& computed_plan, util::undirected_vertex_index_t source)
            {
                if( source == v_source )
                    get_plan(computed_plan, p_source);
                else
                {
                    get_plan(computed_plan, p_target);
                }
            }

            std::string pebble_edge_t::print_constraints() const
            {
                std::stringstream output(std::stringstream::out);

                foreach(unsigned i, constraints)
                {
                    output << i << " , ";
                }
                return output.str();
            }

            bool pebble_edge_t::operator<(const pebble_edge_t& other) const
            {
                return index < other.index;
            }

        }
    }
}


