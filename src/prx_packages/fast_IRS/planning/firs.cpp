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

#include "planning/firs.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/motion_planners/irs/irs.hpp"
#include "prx/planning/motion_planners/irs/irs_statistics.hpp"

#include <boost/property_map/vector_property_map.hpp>
#include <boost/graph/connected_components.hpp>

#include <pluginlib/class_list_macros.h> 


PLUGINLIB_EXPORT_CLASS( prx::packages::fast_irs::FIRS_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    
    namespace packages
    {
        namespace fast_irs
        {
            FIRS_t::FIRS_t()
            {
                random_point = NULL;
                update_k(0);
                num_edges = 0;
                num_vertices = 0;
                num_generated = 0;
                last_solution_length = PRX_INFINITY;
                pno_mode = false;
                
                rejected_edges = 0;
                statistics = new irs_statistics_t();

                astar = fast_astar = new fast_irs_search_t();
            }
            
            FIRS_t::~FIRS_t()
            {
            }

            void FIRS_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                irs_t::init( reader, template_reader );
            }

            void FIRS_t::link_node_to_neighbors(undirected_vertex_index_t v, const std::vector< const abstract_node_t* >& neighbors)
            {
                //fast_astar->set(&graph, metric);
                fast_astar->link_distance_metric(metric);
                fast_astar->link_graph(&graph);
                //fast_astar->clear_structure();
                
//                if( neighbors.size() == 0 )
//                {
//                    PRX_DEBUG_COLOR("Given no neighbors to connect to.", PRX_TEXT_MAGENTA );
//                }
                
                //Attempt to connect to the node's nearest neighbors
                const undirected_node_t* node;
                
                //First of all, I need to know all the distances to these neighbors (for sorting purposes, I might want other things as well)
                std::vector< neighbor_t > Nv;
                for( size_t i=0; i<neighbors.size(); ++i )
                {
                    node = neighbors[i]->as< undirected_node_t >();
                    double dist = metric->distance_function( node->point, graph[v]->point );
                    Nv.push_back( neighbor_t( dist, boost::out_degree( node->index, graph.graph ), node ) );
                }
                std::sort( Nv.begin(), Nv.end() );
                
//                PRX_DEBUG_COLOR("Cleared fast A* structure, node: " << graph[v]->node_id, PRX_TEXT_RED );
//                foreach( const neighbor_t& n, Nv )
//                {
//                    PRX_DEBUG_COLOR("Goal: " << graph[n.node->index]->node_id << " : " << n.distance, PRX_TEXT_LIGHTGRAY );
//                }

                //While there are still neighbors we aren't sure we have the spanner criterion satisfied for
                while( !Nv.empty() )
                {
                    //Get the sorted best neighbor
                    const neighbor_t neighbor = (*Nv.begin());

                    //Remove this guy as a goal
                    //PRX_DEBUG_COLOR("Removing goal: " << graph[Nv.begin()->node->index]->node_id, PRX_TEXT_LIGHTGRAY );
                    Nv.erase( Nv.begin() );

                    path1.clear();
                    new_plan.clear();
                    //Test if we can connect to him
                    local_planner->steer( graph[v]->point, neighbor.node->point, new_plan, path1 );

                    const bool valid = validity_checker->is_valid( path1 );
                    if( valid )
                    {
                        add_edge( v, neighbor.node->index, neighbor.distance, new_plan, path1 );
                        
                        std::set< neighbor_t > reached;
                        get_reached_goals( Nv, v, reached );
                        
                        foreach( const neighbor_t& r, reached )
                        {
                            const neighbor_t n( r );
                            std::vector< neighbor_t >::iterator loc = std::find( Nv.begin(), Nv.end(), n );
                            //PRX_DEBUG_COLOR("Removing goal: " << graph[r.node->index]->node_id, PRX_TEXT_LIGHTGRAY );
                            Nv.erase( loc );
                            ++rejected_edges;
                        }
                    }
                    else
                    {
                        //PRX_DEBUG_COLOR("Edge rejected: " << graph[v]->node_id << " -- " << graph[neighbor.node->index]->node_id, PRX_TEXT_BROWN );
                        ++rejected_edges;
                    }
                }
                fast_astar->clear_structure();
            }
            
            void FIRS_t::add_edge( undirected_vertex_index_t a, undirected_vertex_index_t b, double dist, const plan_t& plan, const trajectory_t& path )
            {
                if( !boost::edge( a, b, graph.graph ).second )
                {
                    //PRX_DEBUG_COLOR("Added edge: " << graph[a]->node_id << " -- " << graph[b]->node_id, PRX_TEXT_CYAN );
                    undirected_edge_index_t e = graph.add_edge< prm_star_edge_t >( a, b, dist );
                    graph.get_edge_as< prm_star_edge_t >(e)->init( control_space, plan, path );
                    graph.get_edge_as< prm_star_edge_t >(e)->id = num_edges++;
                }
            }
            
            void FIRS_t::get_reached_goals( const std::vector< neighbor_t >& Nx, const undirected_vertex_index_t& s, std::set< neighbor_t >& common )
            {
                if( Nx.size() == 0 )
                    return;
                
                //Alright, so all I have to do here is invoke the multi-solve, and use that neighborhood instead.
                std::vector< undirected_vertex_index_t > neighbors;
                
                //PRX_DEBUG_COLOR("About to process the following goals: ", PRX_TEXT_RED );
                foreach( const neighbor_t& n, Nx )
                {
                    neighbors.push_back( n.node->index );
                    //PRX_DEBUG_COLOR("dist: " << metric->distance_function( graph[s]->point, graph[n.node->index]->point ), PRX_TEXT_LIGHTGRAY );
                }
                
                fast_astar->multi_solve( s, neighbors, stretch_factor );
                
                //PRX_DEBUG_COLOR("Astar found: " << fast_astar->get_reached_goals().size(), PRX_TEXT_CYAN );
                
                foreach( undirected_vertex_index_t v, fast_astar->get_reached_goals() )
                {
                    foreach( const neighbor_t& n, Nx )
                    {
                        if( n.node->index == v )
                        {
                            common.insert( n ); 
                        }
                    }
                }
            }
            
        }
    }
}



