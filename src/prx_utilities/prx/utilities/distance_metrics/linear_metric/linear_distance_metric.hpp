/**
 * @file linear_distance_metric.hpp 
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
#ifndef PRX_LINEAR_DISTANCE_METRIC_HPP
#define PRX_LINEAR_DISTANCE_METRIC_HPP

#include "prx/utilities/distance_metrics/distance_metric.hpp"

namespace prx 
 { 
 namespace util 
 {

/**
 * Nearest neighbor data structure which is internally just a vector of points. Searches are done linearly.
 * 
 * @brief <b> Nearest neighbor data structure which is internally just a vector of points. </b>
 * @author Andrew Dobson
 */
class linear_distance_metric_t : public distance_metric_t
{
    public:
        linear_distance_metric_t( );
        ~linear_distance_metric_t();
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
         * @copydoc distance_metric_t::radius_and_closest_query( const space_point_t*, double, std::vector<const abstract_node_t*>& ) const 
         */  
        unsigned radius_and_closest_query( const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest ) const ;
        
        /**
         * @copydoc distance_metric_t::radius_query( const space_point_t*, double, std::vector<const abstract_node_t*>& ) const 
         */          
        unsigned radius_query( const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest ) const ;

        /**
         * @copydoc distance_metric_t::clear()
         */    
        void clear( );
        
        /**
         * Not necessary for linear_distance_metric_t. Empty implementation.
         * @brief Not necessary for linear_distance_metric_t. Empty implementation.
         */
        void rebuild_data_structure( );

    protected:
        
        /**
         * @brief The vector of stored points.
         */
        std::vector<const abstract_node_t*> points;
};

} 
 }

#endif 