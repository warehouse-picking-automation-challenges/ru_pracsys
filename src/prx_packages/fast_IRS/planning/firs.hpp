/**
 * @file firs.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_FIRS_HPP
#define	PRX_FIRS_HPP


#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/planning/motion_planners/irs/irs.hpp"
#include "planning/fast_irs_search.hpp"

namespace prx
{
    namespace packages
    {
        namespace fast_irs
        {

            struct neighbor_t
            {
                double distance;
                unsigned valence;
                const util::undirected_node_t* node;
                
                neighbor_t() {}
                
                neighbor_t( double dist, unsigned val, const util::undirected_node_t* nd )
                {
                    distance = dist;
                    valence = val;
                    node = nd;
                }
                
                neighbor_t( const neighbor_t& other )
                {
                    distance = other.distance;
                    valence = other.valence;
                    node = other.node;
                }
                
                bool operator<( const neighbor_t& other ) const
                {
                    //if( valence == other.valence )
                        return distance < other.distance;
                    //return valence > other.valence;
                }
                
                bool operator==( const neighbor_t& other ) const
                {
                    return node == other.node;
                }
            };
            
            /**
             * Incremental Roadmap Spanner:
             * A method for quickly constructing sparse roadmaps that provide high quality solutions to motion planning queries
             * 
             */
            class FIRS_t : public plan::irs_t
            {
                
            public:
                //    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, irs_node_t, irs_edge_t> util::undirected_graph_t;
                
                FIRS_t();
                virtual ~FIRS_t();
                
                /** @copydoc motion_planner_t::init(const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                
            protected:
                
                virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);
                
                void add_edge( util::undirected_vertex_index_t a, util::undirected_vertex_index_t b, double dist, const sim::plan_t& plan1, const sim::trajectory_t& path1 );

                void get_reached_goals( const std::vector< neighbor_t >& Nx, const util::undirected_vertex_index_t& s, std::set< neighbor_t >& common );
                
                fast_irs_search_t* fast_astar;
            };
            
        }
    }
}

#endif	// PRX_FIRS_HPP

