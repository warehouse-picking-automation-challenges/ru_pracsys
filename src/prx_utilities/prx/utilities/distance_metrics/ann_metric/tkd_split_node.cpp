/**
 * @file tkd_split_node.cpp 
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

#include "tkd_split_node.hpp"

namespace prx 
 { 
 namespace util 
 {

tkd_split_node::tkd_split_node( int cd, ANNcoord cv, ANNcoord lv, ANNcoord hv, ANNkd_ptr lc, ANNkd_ptr hc )
{
    cut_dim	= cd;			// cutting dimension
    cut_val	= cv;			// cutting value
    cd_bnds[LO] = lv;			// lower bound for rectangle
    cd_bnds[HI] = hv;			// upper bound for rectangle
    child[LO]	= lc;			// left (tkd_node_t*)child
    child[HI]	= hc;			// right (tkd_node_t*)child
}

tkd_split_node::tkd_split_node( int cd, ANNcoord cv, ANNcoord lv, ANNcoord hv, int d1, int d2, int d3, ANNcoord lb1, ANNcoord hb1, ANNcoord lb2, ANNcoord hb2, ANNcoord lb3, ANNcoord hb3, ANNkd_ptr lc, ANNkd_ptr hc)
{    	
    cut_dim	= cd;			// cutting dimension
    cut_val	= cv;			// cutting value
    cd_bnds[LO] = lv;			// lower bound for rectangle
    cd_bnds[HI] = hv;			// upper bound for rectangle
    child[LO]	= lc;			// left (tkd_node_t*)child
    child[HI]	= hc;			// right (tkd_node_t*)child
    dim1 = d1;				// rest of the dimensions of P3
    dim2 = d2;
    dim3 = d3;
    bnds1[LO] = lb1;			// bounding values in d1 and d2
    bnds1[HI] = hb1;
    bnds2[LO] = lb2;
    bnds2[HI] = hb2;
    bnds3[LO] = lb3;
    bnds3[HI] = hb3;
}

tkd_split_node::~tkd_split_node( )
{
    if (child[LO]!= NULL && child[LO]!= KD_TRIVIAL) delete child[LO];
    if (child[HI]!= NULL && child[HI]!= KD_TRIVIAL) delete child[HI];
}

void tkd_split_node::ann_search( ANNdist box_dist, const space_t* sp, const std::vector<double*>& sc )
{
    // check dist calc termination condition
    if (ANNmaxPtsVisited && ANNptsVisited > ANNmaxPtsVisited)
        return;
    
    //If the topology is Euclidean, use the standard searching method
    if( sp->get_topology()[cut_dim] == space_t::EUCLIDEAN )
    {
        //Get the raw distance to the cutting plane
        ANNcoord cut_diff = ANNkdQ[cut_dim] - cut_val;

        //If we're on the left side of the cut
        if (cut_diff < 0)
        {
            //Perform search on the lower/left (tkd_node_t*)child
            checked_cast< tkd_node_t*>(child[LO] )->ann_search(box_dist, sp, sc);

            //Get a difference between the lower bound and query point
            ANNcoord box_diff = cd_bnds[LO] - ANNkdQ[cut_dim];
            //Correct if not in bounds
            box_diff = PRX_MAXIMUM( 0, box_diff );
            //Change the distance to the box to be :: b_dist + (b_diff^2 - c_diff^2)
            box_dist = (ANNdist) ANN_SUM(box_dist, ANN_DIFF(ANN_POW(box_diff), ANN_POW(cut_diff)));

            //If the distance times error threshold is less than max key
            if (box_dist * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                //Then perform the search on the right (tkd_node_t*)child
                checked_cast< tkd_node_t*>( child[HI] )->ann_search(box_dist, sp, sc);
        }
        //Otherwise if we're on the right side of the cut
        else
        {
            //Perform search on the higher/right (tkd_node_t*)child
            checked_cast< tkd_node_t*>(child[HI] )->ann_search(box_dist, sp, sc);

            //Get a difference between the query point and the upper bound
            ANNcoord box_diff = ANNkdQ[cut_dim] - cd_bnds[HI];
            //Correct if not in bounds
            box_diff = PRX_MAXIMUM( 0, box_diff );
            //Change the distance to the box to be :: b_dist + (b_diff^2 - c_diff^2)
            box_dist = (ANNdist) ANN_SUM(box_dist, ANN_DIFF(ANN_POW(box_diff), ANN_POW(cut_diff)));
            
            //If the distance times the error threshold is less than max key
            if (box_dist * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                //Then perform the search on the left (tkd_node_t*)child
                checked_cast< tkd_node_t*>(child[LO] )->ann_search(box_dist, sp, sc);
        }
    }
    //If the topology is a sphereical one, then perform a specialized search.
    else if(sp->get_topology()[cut_dim] == space_t::ROTATIONAL )
    {
        ANNdist box_dist1, box_dist2;
        
        // distance to cutting plane
        ANNcoord cut_diff1 = ANNkdQ[cut_dim] - cut_val;

        //
        if (cut_diff1 < 0)       		// left of cutting plane
        {
            ANNcoord cut_diff2 = 2*PI*(*(sp->get_scales()[cut_dim])) + (ANNkdQ[cut_dim] - cut_val);
            
            ANNcoord box_diff1 = cd_bnds[LO] - ANNkdQ[cut_dim];
            box_diff1 = PRX_MAXIMUM( 0, box_diff1 );
            
            ANNcoord box_diff2 = 2*PI*(*(sp->get_scales()[cut_dim])) - (cd_bnds[HI] - ANNkdQ[cut_dim]);
            box_diff2 = PRX_MAXIMUM( 0, box_diff2 );
            
            if (box_diff1 < box_diff2) 
            {
                checked_cast< tkd_node_t*>(child[LO] )->ann_search(box_dist, sp, sc);// visit closer (tkd_node_t*)child first
                // distance to further box
                box_dist1 = (ANNdist) ANN_SUM(box_dist, ANN_DIFF(ANN_POW(box_diff1), ANN_POW(cut_diff1)));
         
                // distance to further box
                box_dist2 = (ANNdist) ANN_SUM(box_dist, ANN_DIFF(ANN_POW(box_diff1), ANN_POW(box_diff2)));
                if (box_dist1 < box_dist2) 
                {
                    // visit further (tkd_node_t*)child if close enough
                    if (box_dist1 * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                        checked_cast< tkd_node_t*>(child[HI] )->ann_search(box_dist1, sp, sc);
                }
                else
                {
                    if (box_dist2 * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                        checked_cast< tkd_node_t*>(child[HI] )->ann_search(box_dist2, sp, sc);
                }
            }
            else 
            {
                checked_cast< tkd_node_t*>(child[HI] )->ann_search(box_dist, sp, sc);// visit closer (tkd_node_t*)child first
                // distance to further box
                box_dist1 = (ANNdist) ANN_SUM(box_dist, ANN_DIFF(ANN_POW(box_diff2), ANN_POW(cut_diff2)));
                
                // distance to further box
                box_dist2 = (ANNdist) ANN_SUM(box_dist, ANN_DIFF(ANN_POW(box_diff2), ANN_POW(box_diff1)));
                if (box_dist1 < box_dist2)
                {
                    // visit further (tkd_node_t*)child if close enough
                    if (box_dist1 * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                        checked_cast< tkd_node_t*>(child[LO] )->ann_search(box_dist1, sp, sc);
                }
                else 
                {
                    if (box_dist2 * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                        checked_cast< tkd_node_t*>(child[LO] )->ann_search(box_dist2, sp, sc);
                }
            }
        }
        else				// right of cutting plane
        {
            ANNcoord cut_diff2 = 2*PI*(*(sp->get_scales()[cut_dim])) - (ANNkdQ[cut_dim] - cut_val);
            
            ANNcoord box_diff1 =  ANNkdQ[cut_dim] - cd_bnds[HI];
            box_diff1 = PRX_MAXIMUM( 0, box_diff1 );
            
            ANNcoord box_diff2 = 2*PI*(*(sp->get_scales()[cut_dim])) - (ANNkdQ[cut_dim] - cd_bnds[LO]);
            box_diff2 = PRX_MAXIMUM( 0, box_diff2 );
            
            if (box_diff1 < box_diff2)
            {
                checked_cast< tkd_node_t*>(child[HI] )->ann_search(box_dist, sp, sc);// visit closer (tkd_node_t*)child first
                // distance to further box
                box_dist1 = (ANNdist) ANN_SUM(box_dist, ANN_DIFF(ANN_POW(box_diff1), ANN_POW(cut_diff1)));
                
                // distance to further box
                box_dist2 = (ANNdist) ANN_SUM(box_dist, ANN_DIFF(ANN_POW(box_diff1), ANN_POW(box_diff2)));
                if (box_dist1 < box_dist2)
                {
                    // visit further (tkd_node_t*)child if close enough
                    if (box_dist1 * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                        checked_cast< tkd_node_t*>(child[LO] )->ann_search(box_dist1, sp, sc);
                }
                else
                {
                    if (box_dist2 * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                        checked_cast< tkd_node_t*>(child[LO] )->ann_search(box_dist2, sp, sc);
                }
            }
            else
            {
                checked_cast< tkd_node_t*>(child[LO] )->ann_search(box_dist, sp, sc);// visit closer (tkd_node_t*)child first
                // distance to further box
                box_dist1 = (ANNdist) ANN_SUM(box_dist, ANN_DIFF(ANN_POW(box_diff2), ANN_POW(cut_diff2)));
                
                
                // distance to further box
                box_dist2 = (ANNdist) ANN_SUM(box_dist, ANN_DIFF(ANN_POW(box_diff2), ANN_POW(box_diff1)));
                if (box_dist1 < box_dist2)
                {
                    // visit further (tkd_node_t*)child if close enough
                    if (box_dist1 * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                        checked_cast< tkd_node_t*>(child[HI] )->ann_search(box_dist1, sp, sc);
                }
                else 
                {
                    if (box_dist2 * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                        checked_cast< tkd_node_t*>(child[HI] )->ann_search(box_dist2, sp, sc);
                }
            }
        }
    }
    //If the topology is a quaternion coordinate, then do a different specialized search
    else
    {
        ANNdist box_dist1, box_dist2;
        
        tkd_rectangle *RectLO = new tkd_rectangle(cd_bnds[LO], cut_val, bnds1[LO], bnds1[HI], bnds2[LO], bnds2[HI], bnds3[LO], bnds3[HI]);
        tkd_rectangle *RectHI = new tkd_rectangle(cut_val, cd_bnds[HI], bnds1[LO], bnds1[HI], bnds2[LO], bnds2[HI], bnds3[LO], bnds3[HI]);
        ANNdist distLO, distLO1, distHI, distHI1;
        distLO = RectLO->DistPointRectangle (ANNkdQ[cut_dim], ANNkdQ[dim1], ANNkdQ[dim2], ANNkdQ[dim3]);
        distLO1 = RectLO->DistPointRectangle (-ANNkdQ[cut_dim], -ANNkdQ[dim1], -ANNkdQ[dim2], -ANNkdQ[dim3]);
        
        if (distLO1 < distLO) 
            distLO = distLO1;
        
        distHI = RectHI->DistPointRectangle (ANNkdQ[cut_dim], ANNkdQ[dim1], ANNkdQ[dim2], ANNkdQ[dim3]);
        distHI1 = RectHI->DistPointRectangle (-ANNkdQ[cut_dim], -ANNkdQ[dim1], -ANNkdQ[dim2], -ANNkdQ[dim3]);
        
        if (distHI1 < distHI)
            distHI = distHI1;
        
        if (distLO < distHI)
        {
            box_dist1 = box_dist;
            box_dist2 = box_dist - distLO + distHI;
            checked_cast< tkd_node_t*>(child[LO] )->ann_search(box_dist1, sp, sc);
            
            if (box_dist2 * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                checked_cast< tkd_node_t*>(child[HI] )->ann_search(box_dist2, sp, sc);
        }
        else 
        {
            box_dist1 = box_dist;
            box_dist2 = box_dist - distHI + distLO;
            checked_cast< tkd_node_t*>(child[HI] )->ann_search(box_dist1, sp, sc);
            
            if (box_dist2 * (ANNkdMaxErr)*(*(sc[cut_dim]))*(*(sc[cut_dim])) < ANNkdPointMK->max_key())
                checked_cast< tkd_node_t*>(child[LO] )->ann_search(box_dist2, sp, sc);
        }
        delete RectLO;
        delete RectHI;
    }
}

std::ostream& operator<<(std::ostream& out, const tkd_split_node& t)
{
    
    out << "Split " << t.cut_dim << " at : " << t.cut_val << std::endl;
    if (t.child[LO]!= NULL && t.child[LO]!= KD_TRIVIAL)
    {
        out << t.child[LO];
    }
    if (t.child[HI]!= NULL && t.child[HI]!= KD_TRIVIAL)
    {
        out << t.child[HI];
    }
    
    return out;
}

} 
 }
