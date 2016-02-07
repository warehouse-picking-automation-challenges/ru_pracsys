/**
 * @file tkd_node.cpp 
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
 
#include "tkd_node.hpp"

namespace prx 
 { 
 namespace util 
 {

tkd_rectangle::tkd_rectangle( ANNcoord lb1, ANNcoord hb1, ANNcoord lb2, ANNcoord hb2, ANNcoord lb3, ANNcoord hb3, ANNcoord lb4, ANNcoord hb4)
{
    bnds1[LO] = lb1;
    bnds1[HI] = hb1;
    bnds2[LO] = lb2;
    bnds2[HI] = hb2;
    bnds3[LO] = lb3;
    bnds3[HI] = hb3;
    bnds4[LO] = lb4;
    bnds4[HI] = hb4;
}

tkd_rectangle::~tkd_rectangle()			// destructor
{
}

ANNdist tkd_rectangle::DistPointRectangle (ANNcoord p1, ANNcoord p2, ANNcoord p3, ANNcoord p4)
{
    ANNdist dist = 0.0;
    ANNdist *d = new ANNdist [4];
 	
    if ( p1 < bnds1[LO] )
        d[0] = bnds1[LO] - p1;
    else if ( p1 < bnds1[HI] )
        d[0] = 0.0;
    else 
        d[0] = bnds1[HI] - p1;
 	
    if ( p2 < bnds2[LO] )
        d[1] = bnds2[LO] - p2;
    else if ( p2 < bnds2[HI] )
        d[1] = 0.0;
    else 
        d[1] = bnds2[HI] - p2;
 	
    if ( p3 < bnds3[LO] )
        d[2] = bnds3[LO] - p3;
    else if ( p3 < bnds3[HI] )
        d[2] = 0.0;
    else 
        d[2] = bnds3[HI] - p3;
 	
    if ( p4 < bnds4[LO] )
        d[3] = bnds4[LO] - p4;
    else if ( p4 < bnds4[HI] )
        d[3] = 0.0;
    else 
        d[3] = bnds4[HI] - p4;
    
    dist = ANN_POW(d[0]) + ANN_POW(d[1]) + ANN_POW(d[2]) + ANN_POW(d[3]);
    delete [] d;
    return dist;
}

ANNdist tkd_box_distance( const ANNpoint q, const ANNpoint lo, const ANNpoint hi, int dim, const space_t* sp )
{
    register ANNdist dist = 0.0;	// sum of squared distances
    register ANNdist t, t1;

    for (register int d = 0; d < dim; d++)
    {
        if ( sp->get_topology()[d] == space_t::ROTATIONAL ) //Rotational
        {
            if (q[d] < lo[d])
            {
                t = ANNdist(lo[d]) - ANNdist(q[d]);
                t1 = 2*PI*(*(sp->get_scales())[d]) - fabs(ANNdist(hi[d]) - ANNdist(q[d]));
                if (t1 < t) t = t1;
                dist = ANN_SUM(dist, ANN_POW(t));
            }
            else if (q[d] > hi[d])
            {
                t = ANNdist(q[d]) - ANNdist(hi[d]);
                t1 = 2*PI*(*(sp->get_scales())[d]) - fabs(ANNdist(q[d]) - ANNdist(lo[d]));
                if (t1 < t) t = t1;
                dist = ANN_SUM(dist, ANN_POW(t));
            }
        }
        else if( sp->get_topology()[d] == space_t::QUATERNION ) //Quaternions
        {
            register tkd_rectangle *Rect = new tkd_rectangle (lo[d], hi[d], lo[d + 1], hi[d + 1], lo[d + 2], hi[d + 2], lo[d + 3], hi[d + 3]);
            t = Rect->DistPointRectangle(q[d], q[d + 1], q[d + 2], q[d + 3]);
            t1 = Rect->DistPointRectangle(-q[d], -q[d + 1], -q[d + 2], -q[d + 3]);
            if (t1 < t) t = t1;
            dist = ANN_SUM(dist, t);

            delete Rect;
            d = d + 3;
        }
        else
        {
            if (q[d] < lo[d])
            {
                t = ANNdist(lo[d]) - ANNdist(q[d]);
                dist = ANN_SUM(dist, ANN_POW(t));
            }
            else if (q[d] > hi[d])
            {
                t = ANNdist(q[d]) - ANNdist(hi[d]);
                dist = ANN_SUM(dist, ANN_POW(t));
            }
        }
    }
    return dist;
}


} 
 }

