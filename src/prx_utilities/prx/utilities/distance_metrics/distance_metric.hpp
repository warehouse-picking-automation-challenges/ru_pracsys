/**
 * @file distance_metric.hpp 
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
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/distance_functions/distance_function.hpp"
#include "prx/utilities/graph/abstract_node.hpp"

#include <pluginlib/class_loader.h>

#ifndef PRX_DISTANCE_METRIC_HPP
#define	PRX_DISTANCE_METRIC_HPP

namespace prx 
 { 
 namespace util 
 {

class parameter_reader_t;

/**
 * An encapsulation of a distance function and a nearest neighbor query structure. 
 * @brief <b> An encapsulation of a distance function and a nearest neighbor query structure. </b>
 * @authors Andrew Dobson, Zakary Littlefield
 */
class distance_metric_t
{
protected:

    /**
     * A struct that details how to perform comparisons between nodes.
     */
    struct compare_node_t
    {
    private:
        /**
         * The distance function to use.
         */
        distance_t func;
        /**
         * The point to compare with.
         */
        const space_point_t* qp;
    public:
        /**
         * @brief Constructor
         * @param in_func The distance function.
         * @param pt The comparison point.
         */
        compare_node_t( distance_t in_func, const space_point_t* pt )
        {
            func = in_func;
            qp = pt;
        }
        
        /**
         * @brief Determines the ordering of two nodes with respect to the query point.
         * @param a First node.
         * @param b Second node.
         * @return If the distance between the point and node a is less than the distance between the point and node b.
         */
        bool operator ()( const abstract_node_t* a, const abstract_node_t* b)
        {
            return func(qp,a->point) < func(qp,b->point) ;
        }
    };
    
public:
    distance_metric_t( );
    virtual ~distance_metric_t(){}
    /**
     * Initializes parameters of the distance metric.
     * @brief Initializes parameters of the distance metric.
     * @param reader The priority reader for reading parameters. If a parameters exists here, it will be used.
     * @param template_reader The backup reader. Any necessary parameters not in reader will be found here.
     */
    void init(const parameter_reader_t * reader, const parameter_reader_t* template_reader = NULL);

    /**
     * Get the current size of the nearest neighbor structure.
     * @brief Get the current size of the nearest neighbor structure.
     * @return The number of points stored.
     */
    unsigned get_nr_points( );
    
    /**
     * Informs the metric about the type of points it is using.
     * @brief Informs the metric about the type of points it is using.
     * @param inspace The space to perform queries in.
     */
    virtual void link_space( const space_t* inspace );
    
    /**
     * Adds a node to the nearest neighbor structure.
     * @brief Adds a node to the nearest neighbor structure.
     * @param embed The node to add.
     * @return The number of points in total.
     */
    virtual unsigned add_point( const abstract_node_t* embed ) = 0;
    
    /**
     * Adds multiple nodes to the nearest neighbor structure.
     * @brief Adds multiple nodes to the nearest neighbor structure.
     * @param embeds The nodes to add.
     * @return The number of points in total.
     */
    virtual unsigned add_points( const std::vector< const abstract_node_t* >& embeds ) = 0;
    
    /**
     * Removes a node for the nearest neighbor structure.
     * @brief Removes a node for the nearest neighbor structure.
     * @param node The node that should be removed.
     */
    virtual void remove_point( const abstract_node_t* node ) = 0;
    
    /**
     * Performs a query for multiple nodes.
     * @brief Performs a query for multiple nodes.
     * @param query_point The point to query around.
     * @param ink The number of closest points to find.
     * @return A vector containing the nodes that are closest to the query point.
     */
    virtual const std::vector< const abstract_node_t* > multi_query( const space_point_t* query_point, unsigned ink ) const = 0;
    
    /**
     * Performs a query for multiple nodes within a radius.
     * @brief Performs a query for multiple nodes within a radius.
     * @param query_point The point to query around.
     * @param rad The radius around the query point to search.
     * @return A vector containing all nodes within the radius around the query point.
     */
    virtual const std::vector< const abstract_node_t* > radius_query( const space_point_t* query_point, double rad )const = 0;
    
    /**
     * Query for the single closest point.
     * @brief Query for the single closest point.
     * @param query_point The point to query around.
     * @return The closest node to the query point.
     */
    virtual const abstract_node_t* single_query( const space_point_t* query_point ) const = 0;
    
    /**
     * Clears the data structure of all data.
     * @brief Clears the data structure of all data.
     */
    virtual void clear( ) = 0;
    
    /**
     * Restructures internal data representations. Necessary for certain internal data representations, but not all.
     * @brief Restructures internal data representations.
     */
    virtual void rebuild_data_structure( ) = 0;
    
    
    /**
     * Performs a query for multiple nodes.
     * @brief Performs a query for multiple nodes.
     * @param query_point The node to query around.
     * @param ink The number of closest points to find.
     * @return A vector containing the nodes that are closest to the query point.
     */
    virtual const std::vector< const abstract_node_t* > multi_query( const abstract_node_t* query_point, unsigned ink ) const
    {
        return multi_query( query_point->point, ink );
    }
    
    /**
     * Performs a query for multiple nodes within a radius.
     * @brief Performs a query for multiple nodes within a radius.
     * @param query_point The node to query around.
     * @param rad The radius around the query point to search.
     * @return A vector containing all nodes within the radius around the query point.
     */
    virtual const std::vector< const abstract_node_t* > radius_query( const abstract_node_t* query_point, const double rad ) const
    {
        return radius_query( query_point->point, rad );
    }
    
    /**
     * Performs a query for multiple nodes within a radius and the closest node.
     * @brief Performs a query for multiple nodes within a radius and the closest node.
     * @param query_point The node to query around.
     * @param rad The radius around the query point to search.
     * @return A vector containing all nodes within the radius around the query point.
     */
    virtual const std::vector< const abstract_node_t* > radius_and_closest_query( const space_point_t* query_point, double rad, const abstract_node_t*& closest ) const 
    {
        //TODO This is not quite correct. Should be pure virtual.
        return radius_query(query_point,rad);
    }
    
    /**
     * Performs a query for multiple nodes within a radius and the closest node. This version requires an allocated vector.
     * @brief Performs a query for multiple nodes within a radius and the closest node.
     * @param query_point The node to query around.
     * @param rad The radius around the query point to search.
     * @param closest A vector that will be populated with all nodes within the radius around the query point.
     * @return The number of points in the vector that are valid.
     */
    virtual unsigned radius_and_closest_query( const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest ) const 
    { 
        //TODO This is not quite correct. Should be pure virtual.
        return 0;
    }\

    /**
     * Query for the single closest point.
     * @brief Query for the single closest point.
     * @param query_point The node to query around.
     * @return The closest node to the query point.
     */
    virtual const abstract_node_t* single_query( const abstract_node_t* query_point ) const
    {
        return single_query(query_point->point);
    }
    
    /**
     * Performs a query for multiple nodes within a radius. This version requires an allocated vector.
     * @brief Performs a query for multiple nodes within a radius.
     * @param query_point The node to query around.
     * @param rad The radius around the query point to search.
     * @param A vector that will be populated with all nodes within the radius around the query point.
     * @return The number of points in the vector that are valid.
     */
    virtual unsigned radius_query( const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest ) const 
    {
        //TODO This is not quite correct. Should be pure virtual.
        return 0;
    } 
    
    /**
     * Prints important information about the distance metric.
     * @brief Prints important information about the distance metric.
     */
    virtual void print()
    {
    }
    
    /**
     * Determines if the metric contains a node that is equivalent to the argument.
     * @brief Determines if the metric contains a node that is equivalent to the argument.
     */
    virtual bool has_point( const abstract_node_t* embed ) 
    { 
        return false; 
    }
    
    /**
     * Used for pluginlib operations. Looks at the plugin xml file for defined classes of this type.
     * @brief Used for pluginlib operations.
     * @return  The class loader from pluginlib.
     */
    static pluginlib::ClassLoader<distance_metric_t>& get_loader();

    /**
     * @brief The distance function to use for distances between points.
     */
    distance_t distance_function;
    
protected:
    /**
     * @brief A constant for the percolation threshold of number of neighbors required for optimal sampling based planners.
     */
    double gamma_val;
    /**
     * @brief A measure of the space being used.
     */
    double space_measure;
    
    /**
     * @brief The number of points currently stored.
     */
    unsigned nr_points;
    
    /**
     * @brief The space that this metric is using.
     */
    const space_t* space;
    
    /**
     * @brief A distance function that could be used.
     */
    distance_function_t* function;
    
    /**
     * @brief The choice of distance function to use.
     */
    std::string function_name;
    
private:
    
    /**
     * @brief The class loader from pluginlib.
     */
    static pluginlib::ClassLoader<distance_metric_t> loader;
};

} 
 }

#endif	

