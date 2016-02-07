/**
 * @file undirected_edge.hpp
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
#ifndef PRX_UNDIRECTED_EDGE_HPP
#define	PRX_UNDIRECTED_EDGE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/abstract_edge.hpp"

namespace prx
{
    namespace util
    {

        /**
         * A base edge class for graph operations on an undirected graph. 
         * 
         * @brief <b> A base edge class for graph operations on an undirected graph. </b>
         */
        class undirected_edge_t : public abstract_edge_t
        {

          public:

            undirected_edge_t(){ }

            virtual ~undirected_edge_t(){ }

            /**
             * @brief The index of this edge in the graph.
             */
            undirected_edge_index_t index;
        };

    }
}

#endif

