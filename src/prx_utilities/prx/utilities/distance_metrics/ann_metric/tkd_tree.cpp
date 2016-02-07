/**
 * @file tkd_tree.cpp 
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

#include "prx/utilities/distance_metrics/ann_metric/tkd_tree.hpp"

namespace prx 
 { 
 namespace util 
 {

tkd_tree_t::tkd_tree_t(int n, int dd, int bs, const space_t* inspace ):ANNkd_tree(n,dd,bs)
{
    space = inspace;
    create_subtypes();
}

tkd_tree_t::tkd_tree_t(ANNpointArray pa, int n, int dd, int bs, const space_t* inspace, ANNsplitRule split):ANNkd_tree(n,dd,bs)
{
    pts = pa;				// where the points are
    if (n == 0) return;			// no points--no sweat

    space = inspace;
    create_subtypes();

    ANNorthRect bnd_box(dd);		// bounding box for points
    annEnclRect(pa, pidx, n, dd, bnd_box);// construct bounding rectangle
    
    bnd_box_lo = annCopyPt(dd, bnd_box.lo);
    bnd_box_hi = annCopyPt(dd, bnd_box.hi);
    
    switch (split)
    {
        case ANN_KD_STD:			// standard kd-splitting rule
            root = rtkd_tree(pa, pidx, n, dd, bs, bnd_box, kd_split, subtypes, inspace);
            break;
        case ANN_KD_MIDPT:			// midpoint split
            root = rtkd_tree(pa, pidx, n, dd, bs, bnd_box, midpt_split, subtypes, inspace);
            break;
        case ANN_KD_FAIR:			// fair split
            root = rtkd_tree(pa, pidx, n, dd, bs, bnd_box, fair_split, subtypes, inspace);
            break;
        case ANN_KD_SUGGEST:		// best (in our opinion)
        case ANN_KD_SL_MIDPT:		// sliding midpoint split
            root = rtkd_tree(pa, pidx, n, dd, bs, bnd_box, sl_midpt_split, subtypes, inspace);
            break;
        case ANN_KD_SL_FAIR:		// sliding fair split
            root = rtkd_tree(pa, pidx, n, dd, bs, bnd_box, sl_fair_split, subtypes, inspace);
            break;
        default:
            annError("Illegal splitting method", ANNabort);
    }
}

tkd_tree_t::~tkd_tree_t()
{
}

int tkd_tree_t::annKSearch(ANNpoint q, int k, ANNidxArray nn_idx, ANNdistArray dd, double eps)
{
    ANNkdDim = dim;			// copy arguments to static equivs
    ANNkdQ = q;
    ANNkdPts = pts;
    ANNptsVisited = 0;			// initialize count of points visited
    
    if (k > n_pts)
    {
        k = n_pts;
        //annError("Requesting more near neighbors than data points", ANNabort);
    }
    
    ANNkdMaxErr = ANN_POW(1.0 + eps);
    //FLOP(2)				// increment floating op count
    
    ANNkdPointMK = new ANNmin_k(k);	// create set for closest k points
    
    checked_cast< tkd_node_t* >(root)->ann_search(tkd_box_distance(q, bnd_box_lo, bnd_box_hi, dim, space), space, space->get_scales() );
    
    for (int i = 0; i < k; i++)
    {
        dd[i] = ANNkdPointMK->ith_smallest_key(i);
        nn_idx[i] = ANNkdPointMK->ith_smallest_info(i);
    }
    delete ANNkdPointMK;		// deallocate closest point set
    return k;
}

void tkd_tree_t::create_subtypes()
{
    unsigned counter = 0;
    for( unsigned i=0; i<space->get_dimension(); ++i )
    {
        if( space->get_topology()[i] == space_t::QUATERNION )
        {
            subtypes.push_back( counter );
            ++counter;
            if( counter == 4 )
                counter = 0;
        }
        else
        {
            subtypes.push_back( 0 );
        }
    }
}

void tkd_tree_t::set_data_start_point( ANNpointArray data_start )
{
    pts = data_start;
}

ANNkd_ptr rtkd_tree( ANNpointArray pa, ANNidxArray pidx, int n, int dim, int bsp, ANNorthRect& bnd_box, ANNkd_splitter splitter, const std::vector< unsigned >& subtypes, const space_t* sp )
{
    ANNkd_ptr ptr;
    int x = 0 , y = 0, z = 0;
    // n small, make a leaf node
    if (n <= bsp)
    {
        if (n == 0)
        {                               // empty leaf node
            return KD_TRIVIAL;		// return (canonical) empty leaf
        }
        else				// construct the node and return
        {
            ptr =  new tkd_leaf_node(n, pidx);
            return ptr;
        }
    }
    // n large, make a splitting node
    else
    {
        int cd;				// cutting dimension
        ANNcoord cv;			// cutting value
        int n_lo;			// number on low side of cut
        ANNkd_node *lo, *hi;		// low and high children
        
        // invoke splitting procedure
        (*splitter)(pa, pidx, bnd_box, n, dim, cd, cv, n_lo);
        
        ANNcoord lv = bnd_box.lo[cd];	// save bounds for cutting dimension
        ANNcoord hv = bnd_box.hi[cd];
        
        bnd_box.hi[cd] = cv;		// modify bounds for left subtree
        lo = rtkd_tree( pa, pidx, n_lo, dim, bsp, bnd_box, splitter, subtypes, sp );
        bnd_box.hi[cd] = hv;		// restore bounds
        
        bnd_box.lo[cd] = cv;		// modify bounds for right subtree
        hi = rtkd_tree( pa, pidx + n_lo, n-n_lo, dim, bsp, bnd_box, splitter, subtypes, sp );
        bnd_box.lo[cd] = lv;		// restore bounds
        
        bool check = sp->get_topology()[cd] == space_t::QUATERNION;
        if( check )
        {
            if( subtypes[cd] == 0 )
            {
                x = cd+1; y = cd+2; z = cd+3;
            }
            else if( subtypes[cd] == 1 )
            {
                x = cd-1; y = cd+1; z = cd+2;
            }
            else if( subtypes[cd] == 2 )
            {
                x = cd-2; y = cd-1; z = cd+1;
            }
            else if( subtypes[cd] == 3 )
            {
                x = cd-3; y = cd-2; z = cd-1;
            }
            ptr = new tkd_split_node( cd, cv, lv, hv, x, y, z, bnd_box.lo[x], bnd_box.hi[x], bnd_box.lo[y], bnd_box.hi[y], bnd_box.lo[z], bnd_box.hi[z], lo, hi );
        }
        else
        {
            ptr = new tkd_split_node(cd, cv, lv, hv, lo, hi);
        }
        
        return ptr;
    }
}

std::ostream& operator<<(std::ostream& out, const tkd_tree_t& t)
{
    if( t.root )
    {
        out << t.root;
    }
    
    return out;
}

} 
 }


