/**
 * @file sirs.cpp
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

#include "planning/sirs.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include <boost/property_map/vector_property_map.hpp>
#include <boost/graph/connected_components.hpp>

#include <pluginlib/class_list_macros.h> 


PLUGINLIB_EXPORT_CLASS( prx::packages::fast_irs::SIRS_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    
    namespace packages
    {
        namespace fast_irs
        {
            SIRS_t::SIRS_t()
            {
            }
            
            SIRS_t::~SIRS_t()
            {
            }

            void SIRS_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                irs_t::init( reader, template_reader );
            }

            void SIRS_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
            {
                //Attempt to connect to the node's nearest neighbors
                const undirected_node_t* node;
                path1.clear();
                
                //First of all, I need to know all the distances to these neighbors (for sorting purposes, I might want other things as well)
                std::set< neighbor_t > Nv;
                for( size_t i=0; i<neighbors.size(); ++i )
                {
                    node = neighbors[i]->as< undirected_node_t >();
                    Nv.insert( neighbor_t( metric->distance_function( node->point, graph[v]->point ), boost::out_degree( node->index, graph.graph ), node ) );
                } //Currently, this will sort by closest first

                PRX_ASSERT( Nv.size() == neighbors.size() );
                
                //While there are still neighbors we aren't sure we have the spanner criterion satisfied for
                while( !Nv.empty() )
                {
                    //Get the sorted best neighbor
                    neighbor_t neighbor = (*Nv.begin());
                    //Now, we never have to consider him again, so remove it from the set
                    Nv.erase( Nv.begin() );
                    
                    new_plan.clear();
                    //Test if we can connect to him
                    local_planner->steer( graph[v]->point, neighbor.node->point, new_plan, path1 );
                    
                    const bool valid = validity_checker->is_valid( path1 );
                    
                    if( valid )
                    {
                        add_edge( v, neighbor.node->index, neighbor.distance, new_plan, path1 );
                        //Rv <- the intersection of neighbor's neighborhood and Nv
                        std::set< neighbor_t > Rv;
                        get_neighborhood_intersection( Nv, neighbor.node->index, Rv );
                        
                        foreach( const neighbor_t& r, Rv )
                        {
                            if( neighbor.distance + metric->distance_function( neighbor.node->point, r.node->point ) < stretch_factor * r.distance )
                            {
                                Nv.erase( r );
                                ++rejected_edges;
                            }
                        }
                    }
                    else
                    {
                        ++rejected_edges;
                    }
                    path1.clear();
                    path2.clear();
                }
            }
            
            void SIRS_t::add_edge( undirected_vertex_index_t a, undirected_vertex_index_t b, double dist, const plan_t& plan1, const trajectory_t& path1 )
            {
                undirected_edge_index_t e = graph.add_edge< prm_star_edge_t >( a, b, dist );
                graph.get_edge_as< prm_star_edge_t >(e)->init( control_space, plan1, path1 );
                graph.get_edge_as< prm_star_edge_t >(e)->id = num_edges++;
            }
            
            void SIRS_t::get_neighborhood_intersection( const std::set< neighbor_t >& Nx, const undirected_vertex_index_t& s, std::set< neighbor_t >& common )
            {
                foreach( undirected_vertex_index_t v, boost::adjacent_vertices( s, graph.graph ) )
                {
                    foreach( const neighbor_t& n, Nx )
                    {
                        if( n.node->index == v ) //If they are the same neighbor
                        {
                            common.insert( n ); //Insert it as a common neighbor
                        }
                    }
                }
            }
            
        }
    }
}



