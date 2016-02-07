/**
 * @file directed_edge.hpp
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
#ifndef PRX_DIRECTED_EDGE_HPP
#define	PRX_DIRECTED_EDGE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/abstract_edge.hpp"


namespace prx
{
    namespace util
    {

        /**
         * A base edge class for graph operations on a directed graph. 
         * 
         * @brief <b> A base edge class for graph operations on a directed graph. </b>
         */
        class directed_edge_t : public abstract_edge_t
        {

          public:

            directed_edge_t(){ }

            virtual ~directed_edge_t(){ }

            /**
             * @brief The index of this edge in the graph.
             */
            directed_edge_index_t index;

        };

    }
}

#endif

