/**
 * @file tkd_leaf_node.cpp 
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

#include "tkd_leaf_node.hpp"

namespace prx 
 { 
 namespace util 
 {

tkd_leaf_node::tkd_leaf_node( int n, ANNidxArray b )
{
    n_pts = n;		// number of points in bucket
    bkt	  = b;		// the bucket
}

tkd_leaf_node::~tkd_leaf_node( )
{
}

void tkd_leaf_node::ann_search( ANNdist box_dist, const space_t* sp, const std::vector<double*>& sc )
{
    register ANNdist dist;      // distance to data point
    register ANNcoord* pp;      // data coordinate pointer
    register ANNcoord* qq;      // query coordinate pointer
    register ANNdist min_dist;  // distance to k-th closest point
    //register int d;
    
    min_dist = ANNkdPointMK->max_key();	// k-th smallest distance so far

    for (int i = 0; i < n_pts; i++)	// check points in bucket
    {
        pp = ANNkdPts[ bkt[i] ];    // first coord of next data point
        qq = ANNkdQ; 	    		// first coord of query point
        dist = 0;
        
        //Using the topology for distance computations
        space_point_t* alpha = sp->alloc_point();
        space_point_t* beta = sp->alloc_point();
        for( unsigned j=0; j<sp->get_dimension(); ++j )
        {
            alpha->at(j) = pp[j];
            beta->at(j) = qq[j];
        }
        dist = ANN_POW( sp->distance( beta, alpha ) );
        sp->free_point( alpha );
        sp->free_point( beta );
      
        if( dist < min_dist && (ANN_ALLOW_SELF_MATCH || dist!=0) )	// and no self-match problem
        {					
            // add it to the list
            ANNkdPointMK->insert(dist, bkt[i]);
            min_dist = ANNkdPointMK->max_key();
        }
    }
    ANNptsVisited += n_pts;		// increment number of points visited
}

std::ostream& operator<<(std::ostream& out, const tkd_leaf_node& t)
{
    for( int i=0; i<t.n_pts; ++i )
    {
        out << t.bkt[i] << std::endl;
    }
    
    return out;
}

} 
 }

