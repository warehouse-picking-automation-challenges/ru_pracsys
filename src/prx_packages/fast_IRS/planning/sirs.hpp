/**
 * @file sirs.hpp
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

#ifndef PRX_SIRS_HPP
#define	PRX_SIRS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/undirected_graph.hpp"
#include "prx/planning/motion_planners/irs/irs.hpp"
//#include "prx/planning/motion_planners/prm_star/prm_star.hpp"

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
                
                bool operator<( const neighbor_t& other ) const
                {
                    //return distance < other.distance;
                    
                    if( valence == other.valence )
                        return distance < other.distance;
                    return valence > other.valence;
                }
            };
            
            /**
             * Incremental Roadmap Spanner:
             * A method for quickly constructing sparse roadmaps that provide high quality solutions to motion planning queries
             * 
             */
            class SIRS_t : public plan::irs_t
            {
                
            public:
                //    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, irs_node_t, irs_edge_t> util::undirected_graph_t;
                
                SIRS_t();
                virtual ~SIRS_t();
                
                /** @copydoc motion_planner_t::init(const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                
            protected:
                
                virtual void link_node_to_neighbors(util::undirected_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors);
                
                void add_edge( util::undirected_vertex_index_t a, util::undirected_vertex_index_t b, double dist, const sim::plan_t& plan1, const sim::trajectory_t& path1 );

                void get_neighborhood_intersection( const std::set< neighbor_t >& Nx, const util::undirected_vertex_index_t& s, std::set< neighbor_t >& common );
            };
            
        }
    }
}

#endif	// PRX_SIRS_HPP

