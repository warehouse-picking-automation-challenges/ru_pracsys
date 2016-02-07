/**
 * @file tkd_split_node.hpp 
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

#ifndef PRX_TKD_SPLIT_NODE
#define PRX_TKD_SPLIT_NODE

#include "ANN.h"
#include "kd_tree.h"
#include "tkd_node.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        // TODO : Fix this so it is tkd_split_node_t <- Add the _t
        
        /**
         * @anchor tkd_split_node
         *
         * This class represents a split node which partitions some space into two halves.
         * These nodes make up the branches of the TKD-tree class.
         *
         * @brief <b> TKD-tree split nodes. </b>
         *
         * @author Andrew Dobson
         */
        class tkd_split_node : public tkd_node_t		// splitting node of a kd-tree
        {
        protected:
            /** @brief The dimension orthogonal to the cutting plane. */
            int             cut_dim;
            /** @brief Location of the cutting plane. */
            ANNcoord		cut_val;
            /** @brief Lower and upper bounds of rectangle along cut dimension. */
            ANNcoord		cd_bnds[2];	
            /** @brief Left and right children of this node. */
            ANNkd_ptr		child[2];
            // for split in P3
            /** @brief The first dimension in P3. */
            int             dim1;
            /** @brief The second dimension in P3. */
            int             dim2;
            /** @brief The third dimension in P3. */
            int             dim3;
            /** @brief Bounding value of the box in the first dimension. */
            ANNcoord		bnds1[2]; 
            /** @brief Bounding value of the box in the second dimension. */
            ANNcoord		bnds2[2];
            /** @brief Bounding value of the box in the third dimension. */
            ANNcoord		bnds3[2];
        public:
            /**
             * @brief Parameterized constructor for the split node.
             *
             * @param cd Cutting Dimension along which to make the cut.
             * @param cv The value where to make the cut.
             * @param lv Lower bound value.
             * @param hv Upper bound value.
             * @param lc Lower child pointer.
             * @param hc Upper child pointer.
             */
            tkd_split_node( int cd, ANNcoord cv, ANNcoord lv, ANNcoord hv, ANNkd_ptr lc=NULL, ANNkd_ptr hc=NULL);
            
            /**
             * @brief Parameterized constructor in P3.
             *
             * @param cd Cutting Dimension along which to make the cut.
             * @param cv The value where to make the cut.
             * @param lv Lower bound value.
             * @param hv Upper bound value.
             * @param d1 First dimension of P3.
             * @param d2 Second dimension of P3.
             * @param d3 Third dimension of P3.
             * @param lb1 Lower bound on first dimension of P3.
             * @param hb1 Upper bound on first dimension of P3.
             * @param lb2 Lower bound on second dimension of P3.
             * @param hb2 Upper bound on second dimension of P3.
             * @param lb3 Lower bound on third dimension of P3.
             * @param hb3 Upper bound on third dimension of P3.
             * @param lc Lower child pointer.
             * @param hc Upper child pointer.
             */
            tkd_split_node( int cd, ANNcoord cv, ANNcoord lv, ANNcoord hv, int d1, int d2, int d3, ANNcoord lb1, ANNcoord hb1, ANNcoord lb2, ANNcoord hb2, ANNcoord lb3, ANNcoord hb3, ANNkd_ptr lc=NULL, ANNkd_ptr hc=NULL);
            
            virtual ~tkd_split_node();				// destructor
            
            /**
             * @copydoc tkd_node_t::ann_search( ANNdist, const space_t*, const std::vector<double*>& )
             */
            virtual void ann_search( ANNdist box_dist, const space_t* sp, const std::vector<double*>& sc );		// standard search routine
            
            void print( int level, ostream& out )
            {
            }
            
            friend std::ostream& operator<<(std::ostream&, const tkd_split_node&);
            
        };
        
        std::ostream& operator<<(std::ostream&, const tkd_split_node&);
        
    }
}

#endif
