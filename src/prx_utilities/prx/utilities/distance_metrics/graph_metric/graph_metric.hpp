/**
 * @file graph_metric.hpp 
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
#ifndef PRX_GRAPH_DISTANCE_METRIC_HPP
#define PRX_GRAPH_DISTANCE_METRIC_HPP

#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/utilities/distance_metrics/graph_metric/graph_proximity.hpp"

namespace prx 
 { 
 namespace util 
 {

/**
 * A distance metric that uses a graph with neighbor information for nearest neighbor queries.
 * @brief <b> A distance metric that uses a graph with neighbor information for nearest neighbor queries. </b>
 * @author Zakary Littlefield
 */
class graph_distance_metric_t : public distance_metric_t
{   
    public:
        graph_distance_metric_t();
        ~graph_distance_metric_t();
        /**
         * @copydoc distance_metric_t::add_point( const abstract_node_t* )
         */
        unsigned add_point( const abstract_node_t* embed );
        
        /**
         * @copydoc distance_metric_t::add_points( const std::vector< const abstract_node_t* >& )
         */
        unsigned add_points( const std::vector< const abstract_node_t* >& embeds );
        
        /**
         * @copydoc distance_metric_t::remove_point( const abstract_node_t* )
         */
        void remove_point( const abstract_node_t* embed );
        
        /**
         * @copydoc distance_metric_t::multi_query( const space_point_t*, unsigned ) const
         */
        const std::vector< const abstract_node_t* > multi_query( const space_point_t* query_point, unsigned ink ) const;
        
        /**
         * @copydoc distance_metric_t::radius_query( const space_point_t*, double ) const
         */
        const std::vector< const abstract_node_t* > radius_query( const space_point_t* query_point, double rad ) const;
        
        /**
         * @copydoc distance_metric_t::radius_and_closest_query( const space_point_t*, double, const abstract_node_t*& ) const 
         */
        const std::vector< const abstract_node_t* > radius_and_closest_query( const space_point_t* query_point, double rad, const abstract_node_t*& closest )const;
        
        /**
         * @copydoc distance_metric_t::single_query( const space_point_t* ) const
         */    
        const abstract_node_t* single_query( const space_point_t* query_point ) const;

        /**
         * No implementation in this class.
         * @brief No implementation in this class.
         */  
        void clear( );
        
        /**
         * Not necessary for graph_distance_metric. Empty implementation.
         * @brief Not necessary for graph_distance_metric. Empty implementation.
         */
        void rebuild_data_structure( );
        
        /**
         * @copydoc distance_metric_t::radius_and_closest_query( const space_point_t*, double, std::vector<const abstract_node_t*>& ) const 
         */ 
        unsigned radius_and_closest_query( const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest ) const ;
        
        /**
         * @copydoc distance_metric_t::radius_query( const space_point_t*, double, std::vector<const abstract_node_t*>& ) const 
         */  
        unsigned radius_query( const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest ) const ;

    protected:

        /**
         * @brief The internal proximity data structure.
         */
        graph_proximity_t* prox;
        
        /**
         * @brief Stores the query_node temporarily.
         */
        abstract_node_t* query_node;
        
        /**
         * @brief Stores the closest nodes temporarily
         */
        proximity_node_t** close_nodes;
        
        /**
         * @brief Stores distances to query points.
         */
        double* distances;
    
    

};

} 
 }

#endif 