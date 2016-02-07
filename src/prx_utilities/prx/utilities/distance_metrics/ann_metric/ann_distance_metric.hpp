#pragma once
/**
 * @file ANN_distance_metric.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2011, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com3
 */

#ifndef ANN_DISTANCE_METRIC_HPP
#define	ANN_DISTANCE_METRIC_HPP

#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/distance_metrics/ann_metric/tkd_tree.hpp"

namespace prx 
 { 
 namespace util 
 {

class space_t;
class space_point_t;

#define PRX_KD_BUCKET             6
#define PRX_KD_SIZE             320
#define PRX_NO_KD_TREES          50
#define PRX_PROXIMITY_GHOST_SIZE 80
#define PRX_KD_INITIAL_DATA_MAX 150

/**
 * A nearest neighbor implementation using Approximate Nearest Neighbors (ANN)
 * @brief <b> A nearest neighbor implementation using Approximate Nearest Neighbors (ANN) </b>
 * @author Andrew Dobson
 */
class ann_distance_metric_t : public distance_metric_t
{
    public:
        ann_distance_metric_t();
        ~ann_distance_metric_t();

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
        const std::vector< const abstract_node_t* > radius_and_closest_query( const space_point_t* query_point, double rad, const abstract_node_t*& closest )const ;
        
        /**
         * @copydoc distance_metric_t::single_query( const space_point_t* ) const
         */        
        const abstract_node_t* single_query( const space_point_t* query_point ) const;
        
        /**
         * Removes temporary memory of removed points and restructures the trees to be balanced.
         * @brief Rebalances tree data structures.
         */
        void rebuild_data_structure( );
        
        /**
         * @copydoc distance_metric_t::clear()
         */   
        void clear( );
                
        /**
         * @copydoc distance_metric_t::has_point()
         */
        bool has_point( const abstract_node_t* embed );
        
        friend std::ostream& operator<<(std::ostream&, const ann_distance_metric_t&);

    protected:
        /**
         * Helper function for showing all points in the structure.
         * @brief Helper function for showing all points in the structure.
         */
        std::string print_points();
        
        /**
         * @brief List of all points added.
         */
        std::vector< const abstract_node_t* > points;
        
        /**
         * @brief List of all the kd_trees.
         */
        std::vector< tkd_tree_t* > trees;
        
        /**
         * @brief Stores offsets for data in the kd_trees.
         */
        std::vector< unsigned > data_start;
        
        /**
         * @brief Stores temporarily removed data.
         */
        std::vector< unsigned > ghost;
        
        /**
         * @brief Raw data for the kd_trees.
         */
        double** data;
        
        /**
         * @brief Number of points in the trees.
         */
        unsigned tree_points;
        
        /**
         * @brief Number of trees
         */
        unsigned nr_trees;
        
        /**
         * @brief Max amount of data.
         */
        unsigned data_max;
};
std::ostream& operator<<(std::ostream&, const ann_distance_metric_t&);
} 
 }

#endif

