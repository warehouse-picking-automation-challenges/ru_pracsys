/**
 * @file undirected_node.hpp
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
#ifndef PRX_UNDIRECTED_NODE_HPP
#define	PRX_UNDIRECTED_NODE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/abstract_node.hpp"
#include <fstream>

#include <boost/graph/adjacency_list.hpp>

namespace prx
{
    namespace util
    {

        /**
         * A base node class for graph operations on an undirected graph. 
         * 
         * @brief <b> A base node class for graph operations on an undirected graph. </b>
         */
        class undirected_node_t : public abstract_node_t
        {

          public:

            undirected_node_t()
            {
                point = NULL;
                graph_direction = UNDIRECTED_NODE;
            }

            /**
             * @brief Constructor
             * @param new_point The point to represent.
             * @param new_index The index of this node in the graph.
             * @param new_id The index number of this node in the graph.
             */
            undirected_node_t(space_point_t* new_point, undirected_vertex_index_t new_index, int new_id)
            {
                point = new_point;
                index = new_index;
                node_id = new_id;
                graph_direction = UNDIRECTED_NODE;
            }

            virtual ~undirected_node_t()
            {
            }

            /**
             * @brief The index of this node in the graph.
             */
            undirected_vertex_index_t index;
        };

    }
}

#endif


