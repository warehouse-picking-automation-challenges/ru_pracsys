/**
 * @file tkd_leaf_node.hpp 
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

#ifndef PRX_TKD_LEAF_NODE
#define PRX_TKD_LEAF_NODE

#include "ANN.h"
#include "kd_search.h"
#include "kd_tree.h"
#include "prx/utilities/math/configurations/vector.hpp"
#include "tkd_node.hpp"

namespace prx 
{ 
    namespace util 
    {
        
        // TODO : Fix this so it's tkd_leaf_node_t <-
        
        /**
         * @anchor tkd_leaf_node
         *
         * Leaf node of a Topological KD tree which stores a bucket of points for which it
         * is responsible for.
         *
         * @brief <b> Topological KD-tree leaf node. </b>
         *
         * @author Andrew Dobson
         */
        class tkd_leaf_node : public tkd_node_t
        {
        protected:
            /** @brief Number of points in this node's bucket */
            int             n_pts;
            /** @brief Pointer to the start of this bucket */
            ANNidxArray     bkt;
            
        public:
            /**
             * @brief Leaf node parameterized constructor.
             *
             * @param n The number of points to put in this node's bucket.
             * @param b The pointer to the bucket of points.
             */
            tkd_leaf_node(int n, ANNidxArray b);
            ~tkd_leaf_node();                   // destructor (none)
            
            /**
             * @copydoc tkd_node_t::ann_search( ANNdist, const space_t*, const std::vector<double*>& )
             */
            virtual void ann_search( ANNdist box_dist, const space_t* sp, const std::vector<double*>& sc );   // standard search
            
            void print( int level, ostream& out )
            {
            }
            
            friend std::ostream& operator<<(std::ostream&, const tkd_leaf_node&);
            
        };
        std::ostream& operator<<(std::ostream&, const tkd_leaf_node&);
    } 
}

#endif
