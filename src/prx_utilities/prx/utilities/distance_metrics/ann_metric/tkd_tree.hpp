/**
 * @file tkd_tree.hpp 
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

#ifndef PRX_TKD_TREE_HPP
#define	PRX_TKD_TREE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "ANN.h"
#include "ANNx.h"
#include "kd_tree.h"
#include "kd_split.h"
#include "prx/utilities/distance_metrics/ann_metric/tkd_leaf_node.hpp"
#include "prx/utilities/distance_metrics/ann_metric/tkd_split_node.hpp"

namespace prx 
 { 
 namespace util 
 {

/**
 * @brief Recursive Topological KD-tree construction method used to build valid KD trees.
 *
 * @param pa The array of all the points.
 * @param pidx A start location to start building the KD buckets from.
 * @param n The number of points to put into this KD tree.
 * @param dim The dimension of the space.
 * @param bsp The maximum allowed bucket size.
 * @param bnd_box Bounding box of the space.
 * @param splitter A splitter method for decomposing the space.
 * @param subtypes Array for quaterion sub-coordinate identification.
 * @param sp Pointer to the PRACSYS space this KD-tree is built over.
 *
 * @return A pointer to the root node of the fully-constructed TKD-tree.
 */
ANNkd_ptr rtkd_tree( ANNpointArray pa, ANNidxArray pidx, int n, int dim, int bsp, ANNorthRect& bnd_box, ANNkd_splitter splitter, const std::vector< unsigned >& subtypes, const space_t* sp = NULL );

/**
 * @anchor tkd_tree_t
 *
 * @brief <b> Topologically-aware KD-tree class. </b>
 *
 * @author Andrew Dobson
 */
class tkd_tree_t : public ANNkd_tree
{
protected:
    /** @brief The space that this tree operates in */
    const space_t* space;  
    /** @brief Subtype indexers for the quaternion coordinates */
    std::vector< unsigned > subtypes; 

public:
    /**
     * @brief TKD-tree simple parameterized constructor.
     *
     * @param n Number of points to put in this tree.
     * @param dd Dimension of the space.
     * @param bs Maximum bucket size.
     * @param inspace The space over which to build the tree.
     */
    tkd_tree_t(int n, int dd, int bs = 1, const space_t* inspace = NULL);
    /**
     * @brief TKD-tree complex parameterized constructor.
     *
     * @param pa The array of all points.
     * @param n Number of points to put in this tree.
     * @param dd Dimension of the space.
     * @param bs Maximum bucket size.
     * @param inspace The space over which to build the tree.
     * @param split The splitting rule to use while constructing.
     */
    tkd_tree_t(ANNpointArray pa, int n, int dd, int bs = 1, const space_t* inspace = NULL, ANNsplitRule split = ANN_KD_SUGGEST);

    ~tkd_tree_t();			// tree destructor

    /**
     * Performs a query on the TKD-tree.  The search attempts to find k points
     * closest to the given query point, q.
     *
     * @brief K-closest neighbor query function.
     *
     * @param q The query point to find the nearest neighbor of.
     * @param k How many nearest neighbors to find.
     * @param nn_idx Nearest neighbors data returned.
     * @param dd Dimension of the space.
     * @param eps Error tolerance value trading accuracy for speed.
     */
    int annKSearch(ANNpoint q, int k, ANNidxArray nn_idx, ANNdistArray dd, double eps=0.0);
    /**
     * Iterate over the topology of the space to determine sub-indicies for
     * quaternion coordinates.  Because there are inherent constraints between
     * the different coordinates of a quaternion, the KD tree must carefully
     * consider the effects on the other coordinates when splitting along a
     * quaternion coordinate.  The constructed subtypes allow the TKD-tree to
     * properly handle such situations.
     *
     * @brief Construct quaternion sub-indices.
     */
    void create_subtypes();
    /**
     * @brief Set the tree's data pointer to point at a specific location in the data array.
     *
     * @param data_start The pointer to have the tree point at.
     */
    void set_data_start_point( ANNpointArray data_start );
    
    friend std::ostream& operator<<(std::ostream&, const tkd_tree_t&);
};
std::ostream& operator<<(std::ostream&, const tkd_tree_t&);
} 
 }

#endif


