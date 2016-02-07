/**
 * @file tkd_node.hpp 
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

#ifndef PRX_TKD_NODE
#define PRX_TKD_NODE

#include "prx/utilities/definitions/defs.hpp"
#include "ANN.h"
#include "pr_queue_k.h"
#include "kd_search.h"
#include "prx/utilities/spaces/space.hpp"
 
#define PI 3.1415926535897932385 //TODO : FOR THE LOVE OF GOD WHY!?

namespace prx 
 { 
 namespace util 
 {

/**
 * @anchor tkd_node_t
 *
 * An abstract class representing a node in a Topological KD space decomposition
 * tree.  
 *
 * @brief <b> Topological KD-tree node. </b>
 *
 * @author Andrew Dobson
 */
class tkd_node_t : public ANNkd_node
{
public:
    tkd_node_t(){};
    ~tkd_node_t(){};

    /**
     * Performs a recursive search down the TKD tree. Populates a list of all
     * potential neighbors for the query point.
     *
     * @brief Recursive tree search.
     *
     * @param box_dist Box distance of the query point from the space where this node exists.
     * @param sp State space where the tree operates.
     * @param sc State space scaling factor.
     */
    virtual void ann_search( ANNdist box_dist, const space_t* sp, const std::vector<double*>& sc ) = 0;

    /**
     * @brief Overload the ann_search method to be empty.
     */
    virtual void ann_search(ANNdist){};	// tree search
    /**
     * @brief Overload the ann_pri_search method to be empty.
     */
    virtual void ann_pri_search(ANNdist){};	// priority search

    /**
     * Empty function that isn't ever overloaded...? TODO : remove the getStats function?
     */
    virtual void getStats( int dim, ANNkdStats &st, ANNorthRect &bnd_box){};
						
    /**
     * Empty function that isn't ever overloaded...? TODO : remove the dump function?
     */
    virtual void dump(ostream &out){};	// dump node

    friend class tkd_tree_t;			// allow tkd-tree to access us
};



/**
 * @anchor tkd_rectangle
 *
 * @brief <b> Rectangle class for representing rectangular space decomposition regions. </b>
 *
 * @author Andrew Dobson
 */
class tkd_rectangle
{
protected:
    /** @brief First dimension bounds. */
    ANNcoord        bnds1[2];		// bounding values
    /** @brief Second dimension bounds. */
    ANNcoord        bnds2[2];		// of the rectangle
    /** @brief Third dimension bounds. */
    ANNcoord        bnds3[2];		// in four dimensions
    /** @brief Fourth dimension bounds. */
    ANNcoord        bnds4[2];
public:
    tkd_rectangle( ANNcoord lb1, ANNcoord hb1, ANNcoord lb2, ANNcoord hb2, ANNcoord lb3, ANNcoord hb3, ANNcoord lb4, ANNcoord hb4 );
    ~tkd_rectangle();			// destructor

    /**
     * @brief Computes a rectangular distance for quaternion coordinates.
     *
     * @param p1 First coordinate value.
     * @param p2 Second coordinate value.
     * @param p3 Third coordinate value.
     * @param p4 Fourth coordinate value.
     *
     * @return The sum of the squred distances along each of the four coordinates.
     */
    ANNdist DistPointRectangle (ANNcoord p1, ANNcoord p2, ANNcoord p3, ANNcoord p4);      
};

/**
 * Computes a box distance from a rectangular region for a given point.
 *
 * @param q The point to check the distance for.
 * @param lo A point representing the lower bounds of the rectangular region.
 * @param hi A point representing the upper bounds of the rectangular region.
 * @param dim The dimensionality of the space.
 * @param sp The space over which this tree is built.
 * @return Sum of squared distances along each dimension between the point and the box.
 */
ANNdist tkd_box_distance( const ANNpoint q, const ANNpoint lo, const ANNpoint hi, int dim, const space_t* sp );

} 
 }

#endif
